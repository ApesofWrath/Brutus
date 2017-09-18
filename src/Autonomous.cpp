/*
 * Autonomous.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: DriversStation
 */

#include <Autonomous.h>
#include <WPILib.h>

const int NUM_POINTS = 1500; //one point for every 10 ms, 15 seconds
const int NUM_INDEX = 15;

const int PISTON_INDEX = 8; //first 7 are for drive
const int FLYWHEEL_INDEX = 9;
const int CONVEYOR_INDEX = 10;
const int ELEVATOR_INDEX = 11; //12 is for vision
const int INTAKE_INDEX = 13;
const int ARM_INDEX = 14; //last column is of 0s

double refs[NUM_POINTS][NUM_INDEX];

DriveController *drive_controller;
Elevator *elevator_au;
Conveyor *conveyor_au;
GearRail *gear_rail_au;
Flywheel *fly_wheel_au;
GroundPickup *ground_pickup_au;

Autonomous::Autonomous(DriveController *drive_controller_pass,
		Elevator *elevator_pass, Conveyor *conveyor_pass,
		GearRail *gear_rail_pass, Flywheel *fly_wheel_pass,
		GroundPickup *ground_pickup_pass) {

	drive_controller = drive_controller_pass;
	elevator_au = elevator_pass;
	conveyor_au = conveyor_pass;
	gear_rail_au = gear_rail_pass;
	fly_wheel_au = fly_wheel_pass;
	ground_pickup_au = ground_pickup_pass;

}

void Autonomous::RunAuton() { // runs continuously through all autonomous modes

	int index = drive_controller->GetIndex(); //sends profile to drive controller

	if (refs[index][PISTON_INDEX] == 1) { //Piston
		gear_rail_au->gear_rail_state = gear_rail_au->open_state_h;
	} else {
		gear_rail_au->gear_rail_state = gear_rail_au->close_state_h;
	}

	if (refs[index][FLYWHEEL_INDEX] == 1) { //Flywheel
		fly_wheel_au->flywheel_state = fly_wheel_au->spin_state_h;
	} else {
		fly_wheel_au->flywheel_state = fly_wheel_au->stop_state_h;
	}

	if (refs[index][CONVEYOR_INDEX] == 1) { //Conveyor
		conveyor_au->conveyor_state = conveyor_au->load_state_h;
	} else {
		conveyor_au->conveyor_state = conveyor_au->stop_state_h;
	}

	if (refs[index][ELEVATOR_INDEX] == 1) { //Elevator
		elevator_au->elevator_state = elevator_au->elevate_state_h;
	} else {
		elevator_au->elevator_state = elevator_au->stop_state_h;
	}

	if (refs[index][ARM_INDEX] == 1) { //Ground Pickup // arm "up" (lower than arm starting position)
		if (refs[index][INTAKE_INDEX] == 2) {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_up_spin_in_state_h;
		} else if (refs[index][INTAKE_INDEX] == 1) {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_up_spin_out_state_h;
		} else {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_up_state_h;
		}
	}

	if (refs[index][ARM_INDEX] == 0) { //Ground Pickup // arm down
		if (refs[index][INTAKE_INDEX] == 2) {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_down_spin_in_state_h;
		} else if (refs[index][INTAKE_INDEX] == 1) {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_down_spin_out_state_h;
		} else {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_down_state_h;
		}
	}

	if (refs[index][ARM_INDEX] == 2) { //Ground Pickup //arm from starting to standard height
		if (refs[index][INTAKE_INDEX] == 2) {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_down_spin_in_state_h;
		//	std::cout << "HERE" << std::endl;

		} else {
			ground_pickup_au->ground_pickup_state =
					ground_pickup_au->arm_start_state_h;
		}
	}

}

void Autonomous::FillProfile(std::string profileName) { //fill array and run auton, extra column of 0s in csv are not carried over into array

	for (int r = 0; r < NUM_POINTS; r++) { //sets the entire array to 0 so that all the points that arent filled are zeros, easy to check for
		for (int c = 0; c < NUM_INDEX; c++) {
			refs[r][c] = 0;
		}
	}

	int r = 0;
	std::fstream file(profileName, std::ios::in);
	while (r < NUM_POINTS) {
		std::string data;
		std::getline(file, data);
		std::stringstream iss(data);
		if (!file.good()) {
			std::cout << "FAIL" << std::endl;
		}
		int c = 0;
		while (c < NUM_INDEX) {
			std::string val;
			std::getline(iss, val, ',');
			std::stringstream convertor(val);
			convertor >> refs[r][c];
			c++;
			//if (file.eof()) {
				//	drive_controller->SetProfileLength(r); //sets array length to length of csv file
			//}
		}
		r++;
	}

	drive_controller->SetRef(refs);
	drive_controller->StartAutonThreads();

}

