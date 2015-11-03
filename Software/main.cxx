/*
 ============================================================================
 Name        : main.c
 Author      : Holly Newton
 Version     :
 Copyright   : Your copyright notice
 Description :
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "props.hxx"
#include "globaldefs.h"
#include "nav/nav_interface.h"
#include "mission/mission_interface.h"
#include "control/control_interface.h"
#include "system_id/systemid_interface.h"
#include "mission/mission_interface.h"

using namespace std;

//only testing nav right now using SGPropertyNode
int main(void)
{
	int autop = 0;
	props = new SGPropertyNode;
	struct  nav   	navData;
	struct sensordata sensorData;
	struct control controlData;
	struct mission missionData;
	double time;
	/*init_dataAq();
	init_sensorProc();*/
	init_mission(/*&sensorData, &navData, &missionData*/);
	init_nav(/*&sensorData, &navData*/);
	/*init_telemetry();
	init_data();*/
	init_control(time, &sensorData, &navData, &controlData, &missionData);
	//init_controlAl();
	init_system_id(/*time, &sensorData, &navData, &controlData, &missionData*/);
	//init_actuators();

	while (1)
	{
		/*get_dataAq();
		get_sensorProc();*/
		get_mission(/*&sensorData, &navData, &missionData*/);
		get_nav(/*&sensorData, &navData*/);
		/*get_telemetry();
		get_data();*/

		if (autop == 1)
		{
			get_control(time, &sensorData, &navData, &controlData, &missionData);
			//get_controlAl();
			get_system_id(/*time, &sensorData, &navData, &controlData, &missionData*/);
			//get_actuators();
		}
	}
}
