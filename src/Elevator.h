/*
 * Elevator.h
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>
#include <CANTalon.h>

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

class Elevator {
public:

	const int stop_state_h = 0;
	const int elevate_state_h = 1;
	const int reverse_state_h = 2;
	int elevator_state = stop_state_h;

	CANTalon *canTalonElevator;

	Elevator();

	void Elevate(double speed);
	void Stop();
	void Reverse(double speed);
	void ElevatorStateMachine();

};

#endif
