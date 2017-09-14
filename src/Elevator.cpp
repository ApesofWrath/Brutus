/*
 * Elevator.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */

#include <Elevator.h>

const int stop_state = 0;
const int elevate_state = 1;
const int reverse_state = 2;
const int CAN_TALON_ELEVATOR = 0;
const double ELEVATE_SPEED = .4;
const double REVERSE_SPEED = .75;

Elevator::Elevator() {

	canTalonElevator = new CANTalon(CAN_TALON_ELEVATOR);

}

void Elevator::Elevate(double speed) {

	canTalonElevator->Set(-speed);

}

void Elevator::Stop() {

	canTalonElevator->Set(0);

}

void Elevator::Reverse(double speed) {

	canTalonElevator->Set(speed);

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case stop_state:

		Stop();

		break;

	case elevate_state:

		Elevate(ELEVATE_SPEED);

		break;

	case reverse_state:

		Reverse(REVERSE_SPEED);

		break;

	}

}
