/*
 * GearRail.h
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>

#ifndef GEARRAIL_H_
#define GEARRAIL_H_

class GearRail {
public:

	const int close_state_h = 0;
	const int open_state_h = 1;
	int gear_rail_state = close_state_h;

	DoubleSolenoid *gearRailPiston;

	GearRail();

	void Open();
	void Close();
	void GearRailStateMachine();

};

#endif
