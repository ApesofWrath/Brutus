
#include <DriveController.h>
#include <fstream>
#include <iostream>
#include <Elevator.h>
#include <Conveyor.h>
#include <GearRail.h>
#include <GroundPickup.h>
#include <Flywheel.h>

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

class Autonomous {
public:

	Autonomous(DriveController *drive_controller_pass, Elevator *elevator_pass, Conveyor *conveyor_pass, GearRail *gear_rail_pass,
			Flywheel *fly_wheel_pass, GroundPickup *ground_pickup_pass);

	void RunAuton();
	void FillProfile(std::string profileName);



};

#endif
