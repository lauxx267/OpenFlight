
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "mission_interface.h"

extern void init_mission(
		struct sensordata *sensorData_ptr,	///< pointer to sensorData structure
		struct nav *navData_ptr,			///< pointer to navData structure
		struct mission *missionData_ptr		///< pointer to missionData structure
){};

extern void get_mission(
		struct sensordata *sensorData_ptr,	///< pointer to sensorData structure
		struct nav *navData_ptr,			///< pointer to navData structure
		struct mission *missionData_ptr		///< pointer to missionData structure
){};

extern void run_mission(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct mission *missionData_ptr) {
	double pilot_mode = sensorData_ptr->inceptorData_ptr->mode;
	double pilot_select = sensorData_ptr->inceptorData_ptr->select;

	if((pilot_mode > 0.5)&&(pilot_mode < 1.5)){
		missionData_ptr -> claw_mode = 1;
	}
	else if(pilot_mode > 1.5){
		missionData_ptr -> claw_mode = 2;
	}
	else{
		missionData_ptr -> claw_mode = 0;
	}

	if((pilot_select > 0.5)&&(pilot_select < 1.5)){
		missionData_ptr -> claw_select = 1;
	}
	else if(pilot_select > 1.5){
		missionData_ptr -> claw_select = 2;
	}
	else{
		missionData_ptr -> claw_select = 0;
	}
}
