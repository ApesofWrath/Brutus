/*
 * GearRail.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */

#include "GearRail.h"
#include <WPILib.h> //TODO: needs to be included in both for reasons i do not know

const int PCM_CAN_ID = 31;

const int EXPAND_CHANNEL = 0;
const int RETRACT_CHANNEL = 1;

const int close_state = 0;
const int open_state = 1;

GearRail::GearRail() {

	gearRailPiston = new DoubleSolenoid(PCM_CAN_ID, EXPAND_CHANNEL, RETRACT_CHANNEL);

}

void GearRail::Open() {

	gearRailPiston->Set(DoubleSolenoid::Value::kForward);

}

void GearRail::Close() {

	gearRailPiston->Set(DoubleSolenoid::Value::kReverse);

}

void GearRail::GearRailStateMachine() {

	switch (gear_rail_state) {

	case close_state:

		Close();

		break;

	case open_state:

		Open();

		break;

	}
}
