#include <Conveyor.h>

const int popcorn_state = 0;
const int load_state = 1;
const int stop_state = 2;
const int CAN_TALON_CONVEYOR = 17;
const double POPCORN_SPEED = .5;
const double LOAD_SPEED = .5;

Conveyor::Conveyor() {

	canTalonConveyor = new CANTalon(CAN_TALON_CONVEYOR);

}

void Conveyor::Popcorn(double speed) {

	canTalonConveyor->Set(-speed);

}

void Conveyor::Stop() {

	canTalonConveyor->Set(0);

}

void Conveyor::Load(double speed) {

	canTalonConveyor->Set(speed);

}

void Conveyor::ConStateMachine() {

	switch (conveyor_state) {

	case popcorn_state:

		Popcorn(POPCORN_SPEED);

		break;

	case load_state:

		Load(LOAD_SPEED);

		break;

	case stop_state:

		Stop();

		break;
	}

}

