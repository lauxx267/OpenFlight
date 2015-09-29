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

int main(void)
{
	int autop;
	init_dataAq();
	init_sensorProc();
	init_mission();
	init_navigation();
	init_telemetry();
	init_data();
	init_ctrlLaw();
	init_controlAl();
	init_systemID();
	init_actuators();

	while (1)
	{
		get_dataAq();
		get_sensorProc();
		get_mission();
		get_navigation();
		get_telemetry();
		get_data();

		if (autop == 1)
		{
			get_ctrlLaw();
			get_controlAl();
			get_systemID();
			get_actuators();
		}
	}
}
