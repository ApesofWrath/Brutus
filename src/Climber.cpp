/*
 * Climber.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: DriversStation
 */

#include <Climber.h>

const int stop_state = 0;
const int climbing_state = 1;
const int CAN_TALON_CLIMBER = 35;
const double CLIMBING_SPEED = .9;

Climber::Climber() { // not climbing back down - sean

	canTalonClimber = new CANTalon(CAN_TALON_CLIMBER);

}

void Climber::Climbing(double speed) {

	canTalonClimber->Set(speed);
}

void Climber::Stop() {

	canTalonClimber->Set(0);

}

double Climber::CheckCurrent(){

	return (double)(canTalonClimber->GetOutputCurrent());


}

void Climber::ClimberStateMachine(){

	switch(climber_state){

	case stop_state:

		Stop();

		break;

	case climbing_state:

		Climbing(CLIMBING_SPEED);

		break;

	}
}




