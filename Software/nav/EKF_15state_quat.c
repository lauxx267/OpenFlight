/*! \file EKF_15state.c
 *	\brief 15 state EKF navigation filter
 *
 *	\details  15 state EKF navigation filter using loosely integrated INS/GPS architecture.
 * 	Time update is done after every IMU data acquisition and GPS measurement
 * 	update is done every time the new data flag in the GPS data packet is set. Designed by Adhika Lie.
 *	Attitude is parameterized using quaternions.
 *	Estimates IMU bias errors.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: EKF_15state_quat.c 911 2012-10-08 15:00:59Z lie $
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
//#include <cyg/posix/pthread.h>
//#include <cyg/kernel/kapi.h
//#include <cyg/cpuload/cpuload.h>

#include "../props.hxx"
//#include "../extern_vars.h"
#include "../utils/matrix.h"
#include "../utils/misc.h"

#include "nav_functions.hxx"
#include "nav_interface.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//error characteristics of navigation parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define		SIG_W_AX	1		//1 m/s^2
#define		SIG_W_AY	1
#define		SIG_W_AZ	1
#define		SIG_W_GX	0.00524		//0.3 deg/s
#define		SIG_W_GY	0.00524
#define		SIG_W_GZ	0.00524
#define		SIG_A_D		0.1	    	//5e-2*g
#define		TAU_A		100
#define		SIG_G_D		0.00873		//0.1 deg/s
#define		TAU_G		50

#define		SIG_GPS_P_NE 3
#define		SIG_GPS_P_D  5
#define		SIG_GPS_V	 0.5

#define		P_P_INIT	10.0
#define		P_V_INIT	1.0
#define		P_A_INIT	0.34906		// 20 deg
#define		P_HDG_INIT	3.14159		//180 deg
#define		P_AB_INIT	0.9810		//0.5*g
#define		P_GB_INIT	0.01745		//5 deg/s

#define     Rew     		6.359058719353925e+006      //earth radius
#define     Rns     		6.386034030458164e+006      //earth radius

/*+++++++++++++++++++++++++++++++++++++++++++++
 * added constant values from global defs
 ++++++++++++++++++++++++++++++++++++++++++++++*/
#define NSECS_PER_SEC	1000000000 		///< [nsec/sec] nanoseconds per second */
const double D2R = 0.017453292519943;	///< [rad] degrees to radians */
#define R2D			57.295779513082323	///< [deg] radians to degrees */
#define PSI_TO_KPA  6.89475729  		///< [KPa] PSI to KPa */
#define	g			9.814				///< [m/sec^2] gravity */
#define g2      	19.62   			///< [m/sec^2] 2*g */
#define PI      	3.14159265358979    ///< pi */
#define PI2     	6.28318530717958	///< pi*2 */
#define half_pi		1.57079632679490	///< pi/2 */
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif


static MATRIX C_N2B, C_B2N;
static MATRIX F, PHI, P, G, K, H;
static MATRIX Rw, Q, Qw, R;
static MATRIX x, y, eul, Rbodtonav, grav, f_b, om_ib, nr;
static MATRIX pos_ref, pos_ins_ecef, pos_ins_ned;
static MATRIX pos_gps, pos_gps_ecef, pos_gps_ned;
static MATRIX I15, I3, ImKH, KRKt;
static MATRIX dx, a_temp31, b_temp31, temp33, atemp33, temp615, temp1515,
		temp66, atemp66, temp156, temp1512;

//static double quat[4];

static double denom, Re, Rn;
static double tprev;

void init_nav() {
	/*++++++++++++++++++++++++++++++++++
	 * Declaring all variables for nav
	 +++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *latitude_nav_rads = fgGetNode("/nav/latitude_nav_rads", true);				///< [rad], geodetic latitude estimate
	SGPropertyNode *longitude_nav_rads = fgGetNode("/nav/longitude_nav_rads", true);			///< [rad], geodetic longitude estimate
	SGPropertyNode *altitude_nav_m = fgGetNode("/nav/altitude_nav_m", true);					///< [m], altitude relative to WGS84 estimate
	SGPropertyNode *northVelocity_nav_mps = fgGetNode("/nav/northVelocity_nav_mps", true);		///< [m/sec], north velocity estimate
	SGPropertyNode *eastVelocity_nav_mps = fgGetNode("/nav/eastVelocity_nav_mps", true);		///< [m/sec], east velocity estimate
	SGPropertyNode *downVelocity_nav_mps = fgGetNode("/nav/downVelocity_nav_mps", true);		///< [m/sec], down velocity estimate
	SGPropertyNode *rollAngle_nav_rads = fgGetNode("/nav/rollAngle_nav_rads", true);			///< [rad], Euler roll angle estimate
	SGPropertyNode *pitchAngle_nav_rads = fgGetNode("/nav/pitchAngle_nav_rads", true);			///< [rad], Euler pitch angle estimate
	SGPropertyNode *yawAngle_nav_rads = fgGetNode("/nav/yawAngle_nav_rads", true);				///< [rad], Euler yaw angle estimate
	SGPropertyNode *quat_nav[4];																///< Quaternions estimate
		quat_nav[0] = fgGetNode("/nav/quat_nav", 0, true);
		quat_nav[1] = fgGetNode("/nav/quat_nav", 1, true);
		quat_nav[2] = fgGetNode("/nav/quat_nav", 2, true);
		quat_nav[3] = fgGetNode("/nav/quat_nav", 3, true);
	SGPropertyNode *accelerometerBias_nav_mpsSq[3];												///< [m/sec^2], accelerometer bias estimate
		accelerometerBias_nav_mpsSq[0] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 0, true);
		accelerometerBias_nav_mpsSq[1] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 1, true);
		accelerometerBias_nav_mpsSq[2] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 2, true);
	SGPropertyNode *gyroBias_nav_rps[3];														///< [rad/sec], rate gyro bias estimate
		gyroBias_nav_rps[0] = fgGetNode("/nav/gyroBias_nav_rps", 0, true);
		gyroBias_nav_rps[1] = fgGetNode("/nav/gyroBias_nav_rps", 1, true);
		gyroBias_nav_rps[2] = fgGetNode("/nav/gyroBias_nav_rps", 2, true);
	SGPropertyNode *covariancePosition_nav_rads[3];												///< [rad], covariance estimate for position
		covariancePosition_nav_rads[0] = fgGetNode("/nav/covariancePosition_nav_rads", 0, true);
		covariancePosition_nav_rads[1] = fgGetNode("/nav/covariancePosition_nav_rads", 1, true);
		covariancePosition_nav_rads[2] = fgGetNode("/nav/covariancePosition_nav_rads", 2, true);
	SGPropertyNode *covarianceVelocity_nav_rads[3];												///< [rad], covariance estimate for velocity
		covarianceVelocity_nav_rads[0] = fgGetNode("/nav/covarianceVelocity_nav_rads", 0, true);
		covarianceVelocity_nav_rads[1] = fgGetNode("/nav/covarianceVelocity_nav_rads", 1, true);
		covarianceVelocity_nav_rads[2] = fgGetNode("/nav/covarianceVelocity_nav_rads", 2, true);
	SGPropertyNode *covarianceAngles_nav_rads[3];												///< [rad], covariance estimate for angles
		covarianceAngles_nav_rads[0] = fgGetNode("/nav/covarianceAngles_nav_rads", 0, true);
		covarianceAngles_nav_rads[1] = fgGetNode("/nav/covarianceAngles_nav_rads", 1, true);
		covarianceAngles_nav_rads[2] = fgGetNode("/nav/covarianceAngles_nav_rads", 2, true);
	SGPropertyNode *covarianceAccelBias_nav_rads[3];											///< [rad], covariance estimate for accelerometer bias
		covarianceAccelBias_nav_rads[0] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 0, true);
		covarianceAccelBias_nav_rads[1] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 1, true);
		covarianceAccelBias_nav_rads[2] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 2, true);
	SGPropertyNode *covarianceGyroBias_nav_rads[3];												///< [rad], covariance estimate for rate gyro bias
		covarianceGyroBias_nav_rads[0] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 0, true);
		covarianceGyroBias_nav_rads[1] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 1, true);
		covarianceGyroBias_nav_rads[2] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 2, true);
	SGPropertyNode *err_type_nav = fgGetNode("/nav/err_type", 0, true); 						///< NAV filter status

	/*+++++++++++++++++++++++++++++++++++++
	 * Declaring variables for imu
	 +++++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *p_imu_rps = fgGetNode("/imu/p_imu_rps", 0, true);								///< [rad/sec], body X axis angular rate (roll)
	SGPropertyNode *q_imu_rps = fgGetNode("/imu/q_imu_rps", 0, true);								///< [rad/sec], body Y axis angular rate (pitch)
	SGPropertyNode *r_imu_rps = fgGetNode("/imu/r_imu_rps", 0, true);								///< [rad/sec], body Z axis angular rate (yaw)
	SGPropertyNode *xAcceleration_imu_mpsSq = fgGetNode("/imu/xAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body X axis acceleration
	SGPropertyNode *yAcceleration_imu_mpsSq = fgGetNode("/imu/yAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body Y axis acceleration
	SGPropertyNode *zAcceleration_imu_mpsSq = fgGetNode("/imu/zAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body Z axis acceleration
	SGPropertyNode *time_imu_sec = fgGetNode("/imu/time_imu_sec", 0, true);							///< [sec], timestamp of IMU data

	/*+++++++++++++++++++++++++++++++++++++
		 * Declaring variables for gps
	 +++++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *latitude_gps_degs = fgGetNode("/gps/latitude_gps_degs", 0, true);				///< [deg], Geodetic latitude
	SGPropertyNode *longitude_gps_degs = fgGetNode("/gps/longitude_gps_degs", 0, true);				///< [deg], Geodetic longitude
	SGPropertyNode *altitude_gps_m = fgGetNode("/gps/altitude_gps_m", 0, true);						///< [m], altitude relative to WGS84
	SGPropertyNode *northVelocity_gps_mps = fgGetNode("/gps/northVelocity_gps_mps", 0, true);		///< [m/sec], North velocity
	SGPropertyNode *eastVelocity_gps_mps = fgGetNode("/gps/eastVelocity_gps_mps", 0, true);			///< [m/sec], East velocity
	SGPropertyNode *downVelocity_gps_mps = fgGetNode("/gps/downVelocity_gps_mps", 0, true);			///< [m/sec], Down velocity


	/*+++++++++++++++++++++++++++++++++++++
		 * Declaring variables for airdata
	 +++++++++++++++++++++++++++++++++++++*/
	/*SGPropertyNode *bias_airdata[10];												///< array for storing biases for air data.
		bias_airdata[0] = fgGetNode("/airdata/bias_airdata", 0, true);
		bias_airdata[1] = fgGetNode("/airdata/bias_airdata", 1, true);
		bias_airdata[2] = fgGetNode("/airdata/bias_airdata", 2, true);
		bias_airdata[3] = fgGetNode("/airdata/bias_airdata", 3, true);
		bias_airdata[4] = fgGetNode("/airdata/bias_airdata", 4, true);
		bias_airdata[5] = fgGetNode("/airdata/bias_airdata", 5, true);
		bias_airdata[6] = fgGetNode("/airdata/bias_airdata", 6, true);
		bias_airdata[7] = fgGetNode("/airdata/bias_airdata", 7, true);
		bias_airdata[8] = fgGetNode("/airdata/bias_airdata", 8, true);
		bias_airdata[9] = fgGetNode("/airdata/bias_airdata", 9, true);
*/

	/*+++++++++++++++++++++++++++++++++++++
		* Declaring variables for enum
	 +++++++++++++++++++++++++++++++++++++*/
		enum   errdefs	{
			data_valid,			///< Data valid
			gps_aided,			///< NAV filter, GPS aided
			TU_only
			};


	/*++++++++++++++++++++++++++++++++++++++++++++++++
	 *matrix creation for navigation computation
	 *++++++++++++++++++++++++++++++++++++++++++++++++*/
	F = mat_creat(15, 15, ZERO_MATRIX);		// State Matrix
	PHI = mat_creat(15, 15, ZERO_MATRIX);		// State transition Matrix

	P = mat_creat(15, 15, ZERO_MATRIX);		// Covariance Matrix
	G = mat_creat(15, 12, ZERO_MATRIX);		// for Process Noise Transformation
	Rw = mat_creat(12, 12, ZERO_MATRIX);
	Qw = mat_creat(15, 15, ZERO_MATRIX);
	Q = mat_creat(15, 15, ZERO_MATRIX);		// Process Noise Matrix
	R = mat_creat(6, 6, ZERO_MATRIX);		// GPS Measurement Noise matrix

	x = mat_creat(15, 1, ZERO_MATRIX);
	y = mat_creat(6, 1, ZERO_MATRIX);		// GPS Measurement
	eul = mat_creat(3, 1, ZERO_MATRIX);
	Rbodtonav = mat_creat(3, 3, ZERO_MATRIX);
	grav = mat_creat(3, 1, ZERO_MATRIX);		// Gravity Model
	f_b = mat_creat(3, 1, ZERO_MATRIX);
	om_ib = mat_creat(3, 1, ZERO_MATRIX);
	nr = mat_creat(3, 1, ZERO_MATRIX);		// Nav Transport Rate

	K = mat_creat(15, 6, ZERO_MATRIX);		// Kalman Gain
	H = mat_creat(6, 15, ZERO_MATRIX);

	C_N2B = mat_creat(3, 3, ZERO_MATRIX);		// DCM
	C_B2N = mat_creat(3, 3, ZERO_MATRIX);		// DCM Transpose

	pos_ref = mat_creat(3, 1, ZERO_MATRIX);
	pos_ins_ecef = mat_creat(3, 1, ZERO_MATRIX);
	pos_ins_ned = mat_creat(3, 1, ZERO_MATRIX);

	pos_gps = mat_creat(3, 1, ZERO_MATRIX);
	pos_gps_ecef = mat_creat(3, 1, ZERO_MATRIX);
	pos_gps_ned = mat_creat(3, 1, ZERO_MATRIX);

	I15 = mat_creat(15, 15, UNIT_MATRIX);		// Identity
	I3 = mat_creat(3, 3, UNIT_MATRIX);		// Identity
	ImKH = mat_creat(15, 15, ZERO_MATRIX);
	KRKt = mat_creat(15, 15, ZERO_MATRIX);

	dx = mat_creat(3, 1, ZERO_MATRIX);		// Temporary to get dxdt
	a_temp31 = mat_creat(3, 1, ZERO_MATRIX);		// Temporary
	b_temp31 = mat_creat(3, 1, ZERO_MATRIX);		// Temporary
	temp33 = mat_creat(3, 3, ZERO_MATRIX);		// Temporary
	atemp33 = mat_creat(3, 3, ZERO_MATRIX);		// Temporary
	temp1515 = mat_creat(15, 15, ZERO_MATRIX);
	temp615 = mat_creat(6, 15, ZERO_MATRIX);
	temp66 = mat_creat(6, 6, ZERO_MATRIX);
	atemp66 = mat_creat(6, 6, ZERO_MATRIX);
	temp156 = mat_creat(15, 6, ZERO_MATRIX);
	temp1512 = mat_creat(15, 12, ZERO_MATRIX);

	// Assemble the matrices
	// .... gravity, g
	grav[2][0] = g;

	// ... H
	H[0][0] = 1.0;
	H[1][1] = 1.0;
	H[2][2] = 1.0;
	H[3][3] = 1.0;
	H[4][4] = 1.0;
	H[5][5] = 1.0;

	// ... Rw
	Rw[0][0] = SIG_W_AX * SIG_W_AX;
	Rw[1][1] = SIG_W_AY * SIG_W_AY;
	Rw[2][2] = SIG_W_AZ * SIG_W_AZ;
	Rw[3][3] = SIG_W_GX * SIG_W_GX;
	Rw[4][4] = SIG_W_GY * SIG_W_GY;
	Rw[5][5] = SIG_W_GZ * SIG_W_GZ;
	Rw[6][6] = 2 * SIG_A_D * SIG_A_D / TAU_A;
	Rw[7][7] = 2 * SIG_A_D * SIG_A_D / TAU_A;
	Rw[8][8] = 2 * SIG_A_D * SIG_A_D / TAU_A;
	Rw[9][9] = 2 * SIG_G_D * SIG_G_D / TAU_G;
	Rw[10][10] = 2 * SIG_G_D * SIG_G_D / TAU_G;
	Rw[11][11] = 2 * SIG_G_D * SIG_G_D / TAU_G;

	// ... P (initial)
	P[0][0] = P_P_INIT * P_P_INIT;
	P[1][1] = P_P_INIT * P_P_INIT;
	P[2][2] = P_P_INIT * P_P_INIT;
	P[3][3] = P_V_INIT * P_V_INIT;
	P[4][4] = P_V_INIT * P_V_INIT;
	P[5][5] = P_V_INIT * P_V_INIT;
	P[6][6] = P_A_INIT * P_A_INIT;
	P[7][7] = P_A_INIT * P_A_INIT;
	P[8][8] = P_HDG_INIT * P_HDG_INIT;
	P[9][9] = P_AB_INIT * P_AB_INIT;
	P[10][10] = P_AB_INIT * P_AB_INIT;
	P[11][11] = P_AB_INIT * P_AB_INIT;
	P[12][12] = P_GB_INIT * P_GB_INIT;
	P[13][13] = P_GB_INIT * P_GB_INIT;
	P[14][14] = P_GB_INIT * P_GB_INIT;

	///****************Changing to SGpropertyNode code
	// ... update P in get_nav
	covariancePosition_nav_rads[0]->setDoubleValue(P[0][0]);
	covariancePosition_nav_rads[1]->setDoubleValue(P[1][1]);
	covariancePosition_nav_rads[2]->setDoubleValue(P[2][2]);
	covarianceVelocity_nav_rads[0]->setDoubleValue(P[3][3]);
	covarianceVelocity_nav_rads[1]->setDoubleValue(P[4][4]);
	covarianceVelocity_nav_rads[2]->setDoubleValue(P[5][5]);
	covarianceAngles_nav_rads[0]->setDoubleValue(P[6][6]);
	covarianceAngles_nav_rads[1]->setDoubleValue(P[7][7]);
	covarianceAngles_nav_rads[2]->setDoubleValue(P[8][8]);

	covarianceAccelBias_nav_rads[0]->setDoubleValue(P[9][9]);
	covarianceAccelBias_nav_rads[1]->setDoubleValue(P[10][10]);
	covarianceAccelBias_nav_rads[2]->setDoubleValue(P[11][11]);
	covarianceGyroBias_nav_rads[0]->setDoubleValue(P[12][12]);
	covarianceGyroBias_nav_rads[1]->setDoubleValue(P[13][13]);
	covarianceGyroBias_nav_rads[2]->setDoubleValue(P[14][14]);

	// ... R
	R[0][0] = SIG_GPS_P_NE * SIG_GPS_P_NE;
	R[1][1] = SIG_GPS_P_NE * SIG_GPS_P_NE;
	R[2][2] = SIG_GPS_P_D * SIG_GPS_P_D;
	R[3][3] = SIG_GPS_V * SIG_GPS_V;
	R[4][4] = SIG_GPS_V * SIG_GPS_V;
	R[5][5] = SIG_GPS_V * SIG_GPS_V;


	// .. then initialize states with GPS Data
	latitude_nav_rads->setDoubleValue(latitude_gps_degs->getDoubleValue()*D2R);
	longitude_nav_rads->setDoubleValue(longitude_gps_degs->getDoubleValue()*D2R);
	altitude_nav_m->setDoubleValue(altitude_gps_m->getDoubleValue());

	northVelocity_nav_mps->setDoubleValue(northVelocity_gps_mps->getDoubleValue());
	eastVelocity_nav_mps->setDoubleValue(eastVelocity_gps_mps->getDoubleValue());
	downVelocity_nav_mps->setDoubleValue(downVelocity_gps_mps->getDoubleValue());

	// ... and initialize states with IMU Data
	pitchAngle_nav_rads->setDoubleValue(asin(xAcceleration_imu_mpsSq->getDoubleValue()/g)); //theta from xacceleration, aircraft at rest
	rollAngle_nav_rads->setDoubleValue(asin((-1*yAcceleration_imu_mpsSq->getDoubleValue())/(g*cos(pitchAngle_nav_rads->getDoubleValue())))); //phi from yacceleration, aircraft at rest
	yawAngle_nav_rads->setDoubleValue(90.0*D2R);

	eul2quat(quat_nav[4], rollAngle_nav_rads, pitchAngle_nav_rads, yawAngle_nav_rads);
	/*eul2quat(navData_ptr->quat, (navData_ptr->phi), (navData_ptr->the),
			(navData_ptr->psi));*/

	accelerometerBias_nav_mpsSq[0]->setDoubleValue(0.0);
	accelerometerBias_nav_mpsSq[1]->setDoubleValue(0.0);
	accelerometerBias_nav_mpsSq[2]->setDoubleValue(0.0);

	gyroBias_nav_rps[0]->setDoubleValue(p_imu_rps->getDoubleValue());
	gyroBias_nav_rps[1]->setDoubleValue(q_imu_rps->getDoubleValue());
	gyroBias_nav_rps[2]->setDoubleValue(r_imu_rps->getDoubleValue());

	// Specific forces and Rotation Rate
	f_b[0][0] = xAcceleration_imu_mpsSq->getDoubleValue() - accelerometerBias_nav_mpsSq[0]->getDoubleValue();
	f_b[1][0] = yAcceleration_imu_mpsSq->getDoubleValue() - accelerometerBias_nav_mpsSq[1]->getDoubleValue();
	f_b[2][0] = zAcceleration_imu_mpsSq->getDoubleValue() - accelerometerBias_nav_mpsSq[2]->getDoubleValue();

	om_ib[0][0] = p_imu_rps->getDoubleValue() - gyroBias_nav_rps[0]->getDoubleValue();
	om_ib[1][0] = q_imu_rps->getDoubleValue() - gyroBias_nav_rps[1]->getDoubleValue();
	om_ib[2][0] = r_imu_rps->getDoubleValue() - gyroBias_nav_rps[2]->getDoubleValue();

	// Time during initialization
	tprev = time_imu_sec->getDoubleValue();

	err_type_nav->setIntValue(data_valid);
	//send_status("NAV filter initialized");
	//fprintf(stderr,"NAV Data Initialization Completed\n");
}

// Main get_nav filter function
void get_nav() {
	double tnow, imu_dt;
	double dq[4], quat_new[4];

	/*++++++++++++++++++++++++++++++++++
	 * Declaring all variables for nav
	 +++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *latitude_nav_rads = fgGetNode("/nav/latitude_nav_rads", true);				///< [rad], geodetic latitude estimate
	SGPropertyNode *longitude_nav_rads = fgGetNode("/nav/longitude_nav_rads", true);			///< [rad], geodetic longitude estimate
	SGPropertyNode *altitude_nav_m = fgGetNode("/nav/altitude_nav_m", true);					///< [m], altitude relative to WGS84 estimate
	SGPropertyNode *northVelocity_nav_mps = fgGetNode("/nav/northVelocity_nav_mps", true);		///< [m/sec], north velocity estimate
	SGPropertyNode *eastVelocity_nav_mps = fgGetNode("/nav/eastVelocity_nav_mps", true);		///< [m/sec], east velocity estimate
	SGPropertyNode *downVelocity_nav_mps = fgGetNode("/nav/downVelocity_nav_mps", true);		///< [m/sec], down velocity estimate
	SGPropertyNode *rollAngle_nav_rads = fgGetNode("/nav/rollAngle_nav_rads", true);			///< [rad], Euler roll angle estimate
	SGPropertyNode *pitchAngle_nav_rads = fgGetNode("/nav/pitchAngle_nav_rads", true);			///< [rad], Euler pitch angle estimate
	SGPropertyNode *yawAngle_nav_rads = fgGetNode("/nav/yawAngle_nav_rads", true);				///< [rad], Euler yaw angle estimate
	SGPropertyNode *quat_nav[4];																///< Quaternions estimate
		quat_nav[0] = fgGetNode("/nav/quat_nav", 0, true);
		quat_nav[1] = fgGetNode("/nav/quat_nav", 1, true);
		quat_nav[2] = fgGetNode("/nav/quat_nav", 2, true);
		quat_nav[3] = fgGetNode("/nav/quat_nav", 3, true);
	SGPropertyNode *accelerometerBias_nav_mpsSq[3];												///< [m/sec^2], accelerometer bias estimate
		accelerometerBias_nav_mpsSq[0] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 0, true);
		accelerometerBias_nav_mpsSq[1] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 1, true);
		accelerometerBias_nav_mpsSq[2] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 2, true);
	SGPropertyNode *gyroBias_nav_rps[3];														///< [rad/sec], rate gyro bias estimate
		gyroBias_nav_rps[0] = fgGetNode("/nav/gyroBias_nav_rps", 0, true);
		gyroBias_nav_rps[1] = fgGetNode("/nav/gyroBias_nav_rps", 1, true);
		gyroBias_nav_rps[2] = fgGetNode("/nav/gyroBias_nav_rps", 2, true);
	SGPropertyNode *covariancePosition_nav_rads[3];												///< [rad], covariance estimate for position
		covariancePosition_nav_rads[0] = fgGetNode("/nav/covariancePosition_nav_rads", 0, true);
		covariancePosition_nav_rads[1] = fgGetNode("/nav/covariancePosition_nav_rads", 1, true);
		covariancePosition_nav_rads[2] = fgGetNode("/nav/covariancePosition_nav_rads", 2, true);
	SGPropertyNode *covarianceVelocity_nav_rads[3];												///< [rad], covariance estimate for velocity
		covarianceVelocity_nav_rads[0] = fgGetNode("/nav/covarianceVelocity_nav_rads", 0, true);
		covarianceVelocity_nav_rads[1] = fgGetNode("/nav/covarianceVelocity_nav_rads", 1, true);
		covarianceVelocity_nav_rads[2] = fgGetNode("/nav/covarianceVelocity_nav_rads", 2, true);
	SGPropertyNode *covarianceAngles_nav_rads[3];												///< [rad], covariance estimate for angles
		covarianceAngles_nav_rads[0] = fgGetNode("/nav/covarianceAngles_nav_rads", 0, true);
		covarianceAngles_nav_rads[1] = fgGetNode("/nav/covarianceAngles_nav_rads", 1, true);
		covarianceAngles_nav_rads[2] = fgGetNode("/nav/covarianceAngles_nav_rads", 2, true);
	SGPropertyNode *covarianceAccelBias_nav_rads[3];											///< [rad], covariance estimate for accelerometer bias
		covarianceAccelBias_nav_rads[0] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 0, true);
		covarianceAccelBias_nav_rads[1] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 1, true);
		covarianceAccelBias_nav_rads[2] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 2, true);
	SGPropertyNode *covarianceGyroBias_nav_rads[3];												///< [rad], covariance estimate for rate gyro bias
		covarianceGyroBias_nav_rads[0] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 0, true);
		covarianceGyroBias_nav_rads[1] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 1, true);
		covarianceGyroBias_nav_rads[2] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 2, true);
	SGPropertyNode *err_type_nav = fgGetNode("/nav/err_type", 0, true); 						///< NAV filter status

	/*+++++++++++++++++++++++++++++++++++++
	 * Declaring variables for imu
	 +++++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *p_imu_rps = fgGetNode("/imu/p_imu_rps", 0, true);							///< [rad/sec], body X axis angular rate (roll)
	SGPropertyNode *q_imu_rps = fgGetNode("/imu/q_imu_rps", 0, true);							///< [rad/sec], body Y axis angular rate (pitch)
	SGPropertyNode *r_imu_rps = fgGetNode("/imu/r_imu_rps", 0, true);							///< [rad/sec], body Z axis angular rate (yaw)
	SGPropertyNode *xAcceleration_imu_mpsSq = fgGetNode("/imu/xAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body X axis acceleration
	SGPropertyNode *yAcceleration_imu_mpsSq = fgGetNode("/imu/yAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body Y axis acceleration
	SGPropertyNode *zAcceleration_imu_mpsSq = fgGetNode("/imu/zAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body Z axis acceleration
	SGPropertyNode *time_imu_sec = fgGetNode("/imu/time_imu_sec", 0, true);							///< [sec], timestamp of IMU data


	/*+++++++++++++++++++++++++++++++++++++
		 * Declaring variables for gps
	 +++++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *latitude_gps_degs = fgGetNode("/gps/latitude_gps_degs", 0, true);				///< [deg], Geodetic latitude
	SGPropertyNode *longitude_gps_degs = fgGetNode("/gps/longitude_gps_degs", 0, true);				///< [deg], Geodetic longitude
	SGPropertyNode *altitude_gps_m = fgGetNode("/gps/altitude_gps_m", 0, true);						///< [m], altitude relative to WGS84
	SGPropertyNode *northVelocity_gps_mps = fgGetNode("/gps/northVelocity_gps_mps", 0, true);									///< [m/sec], North velocity
	SGPropertyNode *eastVelocity_gps_mps = fgGetNode("/gps/eastVelocity_gps_mps", 0, true);									///< [m/sec], East velocity
	SGPropertyNode *downVelocity_gps_mps = fgGetNode("/gps/downVelocity_gps_mps", 0, true);									///< [m/sec], Down velocity
	SGPropertyNode *newData = fgGetNode("/gps/newData", true);

	/*+++++++++++++++++++++++++++++++++++++
		 * Declaring variables for airdata
	 +++++++++++++++++++++++++++++++++++++*/
	SGPropertyNode *bias_airdata[10];												///< array for storing biases for air data.
		bias_airdata[0] = fgGetNode("/airdata/bias_airdata", 0, true);
		bias_airdata[1] = fgGetNode("/airdata/bias_airdata", 1, true);
		bias_airdata[2] = fgGetNode("/airdata/bias_airdata", 2, true);
		bias_airdata[3] = fgGetNode("/airdata/bias_airdata", 3, true);
		bias_airdata[4] = fgGetNode("/airdata/bias_airdata", 4, true);
		bias_airdata[5] = fgGetNode("/airdata/bias_airdata", 5, true);
		bias_airdata[6] = fgGetNode("/airdata/bias_airdata", 6, true);
		bias_airdata[7] = fgGetNode("/airdata/bias_airdata", 7, true);
		bias_airdata[8] = fgGetNode("/airdata/bias_airdata", 8, true);
		bias_airdata[9] = fgGetNode("/airdata/bias_airdata", 9, true);

	/*+++++++++++++++++++++++++++++++++++++
		 * Declaring variables for enum
	 +++++++++++++++++++++++++++++++++++++*/
	enum   errdefs	{
		data_valid,			///< Data valid
		gps_aided,			///< NAV filter, GPS aided
		TU_only
		};

	// compute time-elapsed 'dt'
	// This compute the navigation state at the DAQ's Time Stamp
	tnow = time_imu_sec->getDoubleValue();
	imu_dt = tnow - tprev;
	tprev = tnow;

	// ==================  Time Update  ===================
	// Temporary storage in Matrix form
	/*navData_ptr->quat[0];
	navData_ptr->quat[1];
	navData_ptr->quat[2];
	navData_ptr->quat[3];*/ //is this part necessary?

	a_temp31[0][0] = northVelocity_nav_mps->getDoubleValue();
	a_temp31[1][0] = eastVelocity_nav_mps->getDoubleValue();
	a_temp31[2][0] = downVelocity_nav_mps->getDoubleValue();

	b_temp31[0][0] = latitude_nav_rads->getDoubleValue();
	b_temp31[1][0] = longitude_nav_rads->getDoubleValue();
	b_temp31[2][0] = altitude_nav_m->getDoubleValue();

	// AHRS Transformations
	C_N2B = quat2dcm(quat_nav, C_N2B);
	C_B2N = mat_tran(C_N2B, C_B2N);

	// Attitude Update
	// ... Calculate Navigation Rate
	nr = navrate(a_temp31, b_temp31, nr);

	dq[0] = 1;
	dq[1] = 0.5 * om_ib[0][0] * imu_dt;
	dq[2] = 0.5 * om_ib[1][0] * imu_dt;
	dq[3] = 0.5 * om_ib[2][0] * imu_dt;

	qmult(quat_nav, dq, quat_new);

	quat_nav[0]->setDoubleValue(quat_new[0]
			/ sqrt(
					quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
							+ quat_new[2] * quat_new[2]
							+ quat_new[3] * quat_new[3]));
	quat_nav[1]->setDoubleValue(quat_new[1]
			/ sqrt(
					quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
							+ quat_new[2] * quat_new[2]
							+ quat_new[3] * quat_new[3]));
	quat_nav[2]->setDoubleValue(quat_new[2]
			/ sqrt(
					quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
							+ quat_new[2] * quat_new[2]
							+ quat_new[3] * quat_new[3]));
	quat_nav[3]->setDoubleValue(quat_new[3]
			/ sqrt(
					quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
							+ quat_new[2] * quat_new[2]
							+ quat_new[3] * quat_new[3]));

	if (quat_nav[0] < 0) {
		// Avoid quaternion flips sign
		quat_nav[0]->setDoubleValue(-quat_nav[0]->getDoubleValue());
		quat_nav[1]->setDoubleValue(-quat_nav[1]->getDoubleValue());
		quat_nav[2]->setDoubleValue(-quat_nav[2]->getDoubleValue());
		quat_nav[3]->setDoubleValue(-quat_nav[3]->getDoubleValue());
	}
	/*navData_ptr->quat[0] = quat[0];
	navData_ptr->quat[1] = quat[1];
	navData_ptr->quat[2] = quat[2];
	navData_ptr->quat[3] = quat[3];*/ //is this part necessary?

	quat2eul(quat_nav,rollAngle_nav_rads, pitchAngle_nav_rads, yawAngle_nav_rads);
	//defined in nav_functions
	/*quat2eul(navData_ptr->quat, &(navData_ptr->phi), &(navData_ptr->the),
			&(navData_ptr->psi));*/

	// Velocity Update
	dx = mat_mul(C_B2N, f_b, dx);
	dx = mat_add(dx, grav, dx);
	northVelocity_nav_mps->setDoubleValue(northVelocity_nav_mps->getDoubleValue() + imu_dt * dx[0][0]);
	eastVelocity_nav_mps->setDoubleValue(eastVelocity_nav_mps->getDoubleValue() + imu_dt * dx[1][0]);
	downVelocity_nav_mps->setDoubleValue(downVelocity_nav_mps->getDoubleValue() + imu_dt * dx[2][0]);

	// Position Update
	dx = llarate(a_temp31, b_temp31, dx);
	latitude_nav_rads->setDoubleValue(latitude_nav_rads->getDoubleValue() + imu_dt * dx[0][0]);
	longitude_nav_rads->setDoubleValue(longitude_nav_rads->getDoubleValue() + imu_dt * dx[1][0]);
	altitude_nav_m->setDoubleValue(altitude_nav_m->getDoubleValue() + imu_dt * dx[2][0]);

	// JACOBIAN
	F = mat_fill(F, ZERO_MATRIX);
	// ... pos2gs
	F[0][3] = 1.0;
	F[1][4] = 1.0;
	F[2][5] = 1.0;
	// ... gs2pos
	F[5][2] = -2 * g / EARTH_RADIUS;

	// ... gs2att
	temp33 = sk(f_b, temp33);
	atemp33 = mat_mul(C_B2N, temp33, atemp33);

	F[3][6] = -2.0 * atemp33[0][0];
	F[3][7] = -2.0 * atemp33[0][1];
	F[3][8] = -2.0 * atemp33[0][2];
	F[4][6] = -2.0 * atemp33[1][0];
	F[4][7] = -2.0 * atemp33[1][1];
	F[4][8] = -2.0 * atemp33[1][2];
	F[5][6] = -2.0 * atemp33[2][0];
	F[5][7] = -2.0 * atemp33[2][1];
	F[5][8] = -2.0 * atemp33[2][2];

	// ... gs2acc
	F[3][9] = -C_B2N[0][0];
	F[3][10] = -C_B2N[0][1];
	F[3][11] = -C_B2N[0][2];
	F[4][9] = -C_B2N[1][0];
	F[4][10] = -C_B2N[1][1];
	F[4][11] = -C_B2N[1][2];
	F[5][9] = -C_B2N[2][0];
	F[5][10] = -C_B2N[2][1];
	F[5][11] = -C_B2N[2][2];

	// ... att2att
	temp33 = sk(om_ib, temp33);
	F[6][6] = -temp33[0][0];
	F[6][7] = -temp33[0][1];
	F[6][8] = -temp33[0][2];
	F[7][6] = -temp33[1][0];
	F[7][7] = -temp33[1][1];
	F[7][8] = -temp33[1][2];
	F[8][6] = -temp33[2][0];
	F[8][7] = -temp33[2][1];
	F[8][8] = -temp33[2][2];

	// ... att2gyr
	F[6][12] = -0.5;
	F[7][13] = -0.5;
	F[8][14] = -0.5;

	// ... Accel Markov Bias
	F[9][9] = -1.0 / TAU_A;
	F[10][10] = -1.0 / TAU_A;
	F[11][11] = -1.0 / TAU_A;
	F[12][12] = -1.0 / TAU_G;
	F[13][13] = -1.0 / TAU_G;
	F[14][14] = -1.0 / TAU_G;

	//fprintf(stderr,"Jacobian Created\n");

	// State Transition Matrix: PHI = I15 + F*dt;
	temp1515 = mat_scalMul(F, imu_dt, temp1515);
	PHI = mat_add(I15, temp1515, PHI);

	// Process Noise
	G = mat_fill(G, ZERO_MATRIX);
	G[3][0] = -C_B2N[0][0];
	G[3][1] = -C_B2N[0][1];
	G[3][2] = -C_B2N[0][2];
	G[4][0] = -C_B2N[1][0];
	G[4][1] = -C_B2N[1][1];
	G[4][2] = -C_B2N[1][2];
	G[5][0] = -C_B2N[2][0];
	G[5][1] = -C_B2N[2][1];
	G[5][2] = -C_B2N[2][2];

	G[6][3] = -0.5;
	G[7][4] = -0.5;
	G[8][5] = -0.5;

	G[9][6] = 1.0;
	G[10][7] = 1.0;
	G[11][8] = 1.0;
	G[12][9] = 1.0;
	G[13][10] = 1.0;
	G[14][11] = 1.0;
	//fprintf(stderr,"Process Noise Matrix G is created\n");
	// Discrete Process Noise
	temp1512 = mat_mul(G, Rw, temp1512);
	temp1515 = mat_transmul(temp1512, G, temp1515);	// Qw = G*Rw*G'
	Qw = mat_scalMul(temp1515, imu_dt, Qw);			// Qw = dt*G*Rw*G'
	Q = mat_mul(PHI, Qw, Q);						// Q = (I+F*dt)*Qw

	temp1515 = mat_tran(Q, temp1515);
	Q = mat_add(Q, temp1515, Q);
	Q = mat_scalMul(Q, 0.5, Q);				// Q = 0.5*(Q+Q')
	//fprintf(stderr,"Discrete Process Noise is created\n");

	// Covariance Time Update
	temp1515 = mat_mul(PHI, P, temp1515);
	P = mat_transmul(temp1515, PHI, P); 		// P = PHI*P*PHI'
	P = mat_add(P, Q, P);						// P = PHI*P*PHI' + Q
	temp1515 = mat_tran(P, temp1515);
	P = mat_add(P, temp1515, P);
	P = mat_scalMul(P, 0.5, P);				// P = 0.5*(P+P')
	//fprintf(stderr,"Covariance Updated through Time Update\n");

	covariancePosition_nav_rads[0]->setDoubleValue(P[0][0]);
	covariancePosition_nav_rads[1]->setDoubleValue(P[1][1]);
	covariancePosition_nav_rads[2]->setDoubleValue(P[2][2]);
	covarianceVelocity_nav_rads[0]->setDoubleValue(P[3][3]);
	covarianceVelocity_nav_rads[1]->setDoubleValue(P[4][4]);
	covarianceVelocity_nav_rads[2]->setDoubleValue(P[5][5]);
	covarianceAngles_nav_rads[0]->setDoubleValue(P[6][6]);
	covarianceAngles_nav_rads[1]->setDoubleValue(P[7][7]);
	covarianceAngles_nav_rads[2]->setDoubleValue(P[8][8]);
	covarianceAccelBias_nav_rads[0]->setDoubleValue(P[9][9]);
	covarianceAccelBias_nav_rads[1]->setDoubleValue(P[10][10]);
	covarianceAccelBias_nav_rads[2]->setDoubleValue(P[11][11]);
	covarianceGyroBias_nav_rads[0]->setDoubleValue(P[12][12]);
	covarianceGyroBias_nav_rads[1]->setDoubleValue(P[13][13]);
	covarianceGyroBias_nav_rads[2]->setDoubleValue(P[14][14]);

	err_type_nav->setIntValue(TU_only);
	//fprintf(stderr,"Time Update Done\n");
	// ==================  DONE TU  ===================

	if (newData) {

		// ==================  GPS Update  ===================
		newData->setDoubleValue(0);// Reset the flag

		// Position, converted to NED
		a_temp31[0][0] = latitude_nav_rads->getDoubleValue();
		a_temp31[1][0] = longitude_nav_rads->getDoubleValue();
		a_temp31[2][0] = altitude_nav_m->getDoubleValue();
		pos_ins_ecef=lla2ecef(a_temp31, pos_ins_ecef);

		a_temp31[2][0] = 0.0;
		//pos_ref = lla2ecef(a_temp31,pos_ref);
		pos_ref = mat_copy(a_temp31, pos_ref);
		pos_ins_ned = ecef2ned(pos_ins_ecef, pos_ins_ned, pos_ref);

		pos_gps[0][0] = latitude_gps_degs->getDoubleValue() * D2R;
		pos_gps[1][0] = longitude_gps_degs->getDoubleValue() * D2R;
		pos_gps[2][0] = altitude_gps_m->getDoubleValue();

		pos_gps_ecef = lla2ecef(pos_gps, pos_gps_ecef);

		pos_gps_ned = ecef2ned(pos_gps_ecef, pos_gps_ned, pos_ref);

		// Create Measurement: y
		y[0][0] = pos_gps_ned[0][0] - pos_ins_ned[0][0];
		y[1][0] = pos_gps_ned[1][0] - pos_ins_ned[1][0];
		y[2][0] = pos_gps_ned[2][0] - pos_ins_ned[2][0];

		y[3][0] = northVelocity_gps_mps->getDoubleValue() - northVelocity_nav_mps->getDoubleValue();
		y[4][0] = eastVelocity_gps_mps->getDoubleValue() - eastVelocity_nav_mps->getDoubleValue();
		y[5][0] = downVelocity_gps_mps->getDoubleValue() - downVelocity_nav_mps->getDoubleValue();

		//fprintf(stderr,"Measurement Matrix, y, created\n");

		// Kalman Gain
		temp615 = mat_mul(H, P, temp615);
		temp66 = mat_transmul(temp615, H, temp66);
		atemp66 = mat_add(temp66, R, atemp66);
		temp66 = mat_inv(atemp66, temp66); // temp66 = inv(H*P*H'+R)
		//fprintf(stderr,"inv(H*P*H'+R) Computed\n");

		temp156 = mat_transmul(P, H, temp156); // P*H'
		//fprintf(stderr,"P*H' Computed\n");
		K = mat_mul(temp156, temp66, K);	   // K = P*H'*inv(H*P*H'+R)
		//fprintf(stderr,"Kalman Gain Computed\n");

		// Covariance Update
		temp1515 = mat_mul(K, H, temp1515);
		ImKH = mat_sub(I15, temp1515, ImKH);	// ImKH = I - K*H

		temp615 = mat_transmul(R, K, temp615);
		KRKt = mat_mul(K, temp615, KRKt);		// KRKt = K*R*K'

		temp1515 = mat_transmul(P, ImKH, temp1515);
		P = mat_mul(ImKH, temp1515, P);		// ImKH*P*ImKH'
		temp1515 = mat_add(P, KRKt, temp1515);
		P = mat_copy(temp1515, P);			// P = ImKH*P*ImKH' + KRKt
		//fprintf(stderr,"Covariance Updated through GPS Update\n");

		covariancePosition_nav_rads[0]->setDoubleValue(P[0][0]);
		covariancePosition_nav_rads[1]->setDoubleValue(P[1][1]);
		covariancePosition_nav_rads[2]->setDoubleValue(P[2][2]);
		covarianceVelocity_nav_rads[0]->setDoubleValue(P[3][3]);
		covarianceVelocity_nav_rads[1]->setDoubleValue(P[4][4]);
		covarianceVelocity_nav_rads[2]->setDoubleValue(P[5][5]);
		covarianceAngles_nav_rads[0]->setDoubleValue(P[6][6]);
		covarianceAngles_nav_rads[1]->setDoubleValue(P[7][7]);
		covarianceAngles_nav_rads[2]->setDoubleValue(P[8][8]);
		covarianceAccelBias_nav_rads[0]->setDoubleValue(P[9][9]);
		covarianceAccelBias_nav_rads[1]->setDoubleValue(P[10][10]);
		covarianceAccelBias_nav_rads[2]->setDoubleValue(P[11][11]);
		covarianceGyroBias_nav_rads[0]->setDoubleValue(P[12][12]);
		covarianceGyroBias_nav_rads[1]->setDoubleValue(P[13][13]);
		covarianceGyroBias_nav_rads[2]->setDoubleValue(P[14][14]);

		// State Update
		x = mat_mul(K, y, x);
		denom = (1.0 - (ECC2 * sin(latitude_nav_rads->getDoubleValue()) * sin(latitude_nav_rads->getDoubleValue())));
		denom = sqrt(denom * denom);

		Re = EARTH_RADIUS / sqrt(denom);
		Rn = EARTH_RADIUS * (1 - ECC2) / denom * sqrt(denom);
		altitude_nav_m->setDoubleValue(altitude_nav_m->getDoubleValue() - x[2][0]);
		latitude_nav_rads->setDoubleValue(latitude_nav_rads->getDoubleValue() + x[0][0]/(Re + altitude_nav_m->getDoubleValue()));
		longitude_nav_rads->setDoubleValue(longitude_nav_rads->getDoubleValue() + x[1][0]/(Rn + altitude_nav_m->getDoubleValue())/cos(latitude_nav_rads->getDoubleValue()));

		northVelocity_nav_mps->setDoubleValue(northVelocity_nav_mps->getDoubleValue() + x[3][0]);
		eastVelocity_nav_mps->setDoubleValue(eastVelocity_nav_mps->getDoubleValue() + x[4][0]);
		downVelocity_nav_mps->setDoubleValue(downVelocity_nav_mps->getDoubleValue() + x[5][0]);

		/*quat[0] = navData_ptr->quat[0];
		quat[1] = navData_ptr->quat[1];
		quat[2] = navData_ptr->quat[2];
		quat[3] = navData_ptr->quat[3];*///Is this necessary?

		// Attitude correction
		dq[0] = 1.0;
		dq[1] = x[6][0];
		dq[2] = x[7][0];
		dq[3] = x[8][0];

		qmult(quat_nav, dq, quat_new);

		quat_nav[0]->setDoubleValue(quat_new[0]
				/ sqrt(
						quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
								+ quat_new[2] * quat_new[2]
								+ quat_new[3] * quat_new[3]));
		quat_nav[1]->setDoubleValue(quat_new[1]
				/ sqrt(
						quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
								+ quat_new[2] * quat_new[2]
								+ quat_new[3] * quat_new[3]));
		quat_nav[2]->setDoubleValue(quat_new[2]
				/ sqrt(
						quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
								+ quat_new[2] * quat_new[2]
								+ quat_new[3] * quat_new[3]));
		quat_nav[3]->setDoubleValue(quat_new[3]
				/ sqrt(
						quat_new[0] * quat_new[0] + quat_new[1] * quat_new[1]
								+ quat_new[2] * quat_new[2]
								+ quat_new[3] * quat_new[3]));

		/*navData_ptr->quat[0] = quat[0];
		navData_ptr->quat[1] = quat[1];
		navData_ptr->quat[2] = quat[2];
		navData_ptr->quat[3] = quat[3];*///Is this necessary?

		quat2eul(quat_nav, rollAngle_nav_rads, pitchAngle_nav_rads), yawAngle_nav_rads);
		/*quat2eul(navData_ptr->quat, &(navData_ptr->phi), &(navData_ptr->the),
				&(navData_ptr->psi));*/

		accelerometerBias_nav_mpsSq[0]->setDoubleValue(accelerometerBias_nav_mpsSq[0]->getDoubleValue() + x[9][0]);
		accelerometerBias_nav_mpsSq[1]->setDoubleValue(accelerometerBias_nav_mpsSq[1]->getDoubleValue() + x[10][0]);
		accelerometerBias_nav_mpsSq[2]->setDoubleValue(accelerometerBias_nav_mpsSq[2]->getDoubleValue() + x[11][0]);

		gyroBias_nav_rps[0]->setDoubleValue(gyroBias_nav_rps[0]->getDoubleValue() + x[12][0]);
		gyroBias_nav_rps[1]->setDoubleValue(gyroBias_nav_rps[1]->getDoubleValue() + x[13][0]);
		gyroBias_nav_rps[2]->setDoubleValue(gyroBias_nav_rps[2]->getDoubleValue() + x[14][0]);

		err_type_nav->setIntValue(gps_aided);
		//fprintf(stderr,"Measurement Update Done\n");
	}

	// Remove current estimated biases from rate gyro and accels
	p_imu_rps->setDoubleValue(p_imu_rps->getDoubleValue() - gyroBias_nav_rps[0]->getDoubleValue());
	q_imu_rps->setDoubleValue(q_imu_rps->getDoubleValue() - gyroBias_nav_rps[1]->getDoubleValue());
	r_imu_rps->setDoubleValue(r_imu_rps->getDoubleValue() - gyroBias_nav_rps[2]->getDoubleValue());
	xAcceleration_imu_mpsSq->setDoubleValue(xAcceleration_imu_mpsSq->getDoubleValue() - accelerometerBias_nav_mpsSq[0]->getDoubleValue());
	yAcceleration_imu_mpsSq->setDoubleValue(yAcceleration_imu_mpsSq->getDoubleValue() - accelerometerBias_nav_mpsSq[1]->getDoubleValue());
	zAcceleration_imu_mpsSq->setDoubleValue(zAcceleration_imu_mpsSq->getDoubleValue() - accelerometerBias_nav_mpsSq[2]->getDoubleValue());

	// Get the new Specific forces and Rotation Rate,
	// use in the next time update

	f_b[0][0] = xAcceleration_imu_mpsSq->getDoubleValue();
	f_b[1][0] = yAcceleration_imu_mpsSq->getDoubleValue();
	f_b[2][0] = zAcceleration_imu_mpsSq->getDoubleValue();

	om_ib[0][0] = p_imu_rps->getDoubleValue();
	om_ib[1][0] = q_imu_rps->getDoubleValue();
	om_ib[2][0] = r_imu_rps->getDoubleValue();

}

void close_nav(void) {
	//free memory space
	mat_free(F);
	mat_free(PHI);
	mat_free(P);
	mat_free(G);
	mat_free(Rw);
	mat_free(Q);
	mat_free(Qw);
	mat_free(R);
	mat_free(eul);
	mat_free(x);
	mat_free(y);
	mat_free(Rbodtonav);
	mat_free(grav);
	mat_free(f_b);
	mat_free(om_ib);
	mat_free(nr);
	mat_free(K);
	mat_free(H);
	mat_free(C_N2B);
	mat_free(C_B2N);
	mat_free(pos_ref);
	mat_free(pos_ins_ecef);
	mat_free(pos_ins_ned);
	mat_free(pos_gps);
	mat_free(pos_gps_ecef);
	mat_free(pos_gps_ned);
	mat_free(I15);
	mat_free(I3);
	mat_free(ImKH);
	mat_free(KRKt);
	mat_free(dx);
	mat_free(a_temp31);
	mat_free(b_temp31);
	mat_free(temp33);
	mat_free(atemp33);
	mat_free(temp1515);
	mat_free(temp615);
	mat_free(temp66);
	mat_free(atemp66);
	mat_free(temp156);
	mat_free(temp1512);

}
