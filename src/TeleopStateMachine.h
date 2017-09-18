/*
 * TeleopStateMachine.h
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>
#include <DriveController.h>
#include <Flywheel.h>
#include <Elevator.h>
#include <GearRail.h>
#include <Conveyor.h>
#include <Vision.h>
#include <GroundPickup.h>
#include <Climber.h>
#include <SmartDashboard/SmartDashboard.h>

#ifndef TELEOPSTATEMACHINE_H_
#define TELEOPSTATEMACHINE_H_

class TeleopStateMachine {
public:

	TeleopStateMachine(Flywheel *flywheelP, Conveyor *conveyorP,
			GearRail *gearRailP, Elevator *elevatorP,
			DriveController *driveControllerP, Vision *visionP,
			Climber *climberP, GroundPickup *groundPickupP);
	void StateMachine(bool is_gear, bool is_close_gear, bool is_fire,
			bool is_climb, bool is_ret, bool is_popcorn,
			bool second_fire_button, bool stop_shoot_button,
			bool gear_pickup_button, bool gear_score_button,
			bool is_intake, bool is_outtake, bool is_arm_up, bool is_arm_down, bool is_at_pos);
	void Initialize();

};

#endif
