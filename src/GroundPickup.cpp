/*
 * GearPickup.cpp
 *
 *  Created on: Jul 15, 2017
 *      Author: DriversStation
 */

#include <GroundPickup.h>
#define PI 3.14159265

double DOWN_ANGLE = 0.0;
double STANDARD_ANGLE = 2.2; //"up"
double STARTING_ANGLE = 2.5;
double SPIN_SPEED = 0.35;
double ACCEPTABLE_ERROR = 0.09;

double MAX_OUTPUT = 12.0; //volts
double MIN_OUTPUT = -12.0;

const int PICKUP_SLEEP_TIME = 0;
const double PICKUP_WAIT_TIME = 0.01; //ms

const int ENCODER_TICKS = 4096;
const double NUM_RADS = 2.0 * PI;

const int arm_up_state = 0;
const int spin_out_state = 1;
const int spin_in_state = 2;
const int spin_stop_state = 3;
const int arm_down_state = 4;
const int arm_up_spin_in_state = 5;
const int arm_up_spin_out_state = 6;
const int arm_down_spin_in_state = 7;
const int arm_down_spin_out_state = 8;
const int arm_start_state = 9;
const int arm_start_spin_in_state = 10;

int ground_pickup_state = arm_up_state;

double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

double P = 0.0;
double I = 0.0;
double D = 0.0;

double i = 0.0;
double d = 0.0;

double free_speed = 1967.; //rad/s
double m = 2.1;
double l = 0.2;
double Kt = 0.00595;
double G = ((60.0 / 1.0) * (48.0 / 38.0));

double MAX_THEORETICAL_VELOCITY = (free_speed / G);
double MAX_VELOCITY = 28.0; //choose
double MAX_ACCELERATION = 38.0; //choose
double TIME_STEP = 0.01;
double Kv = 1 / MAX_THEORETICAL_VELOCITY;

double calculatedFeedForward = 0.0;

double position = 0.0;
double error = 0.0;
double output = 0.0;
double last_error = 0.0;
double last_ref = 0.0;

double vel_ref = 0.0;
double pos_ref = 0.0;
double last_vel_ref = 0.0;
double last_pos_ref = 0.0;

int profileIndex = 0;
double profileVelocity = 0.0;

Timer *timerPickup = new Timer();

std::thread PickupThread;

TrapezoidalProfile *intake_profiler = new TrapezoidalProfile(MAX_VELOCITY,
		MAX_ACCELERATION, TIME_STEP);

std::vector<std::vector<double>> down_to_standard_profile =
		intake_profiler->CreateProfile(DOWN_ANGLE, STANDARD_ANGLE);
std::vector<std::vector<double>> standard_to_down_profile =
		intake_profiler->CreateProfile(STANDARD_ANGLE, DOWN_ANGLE);
std::vector<std::vector<double>> starting_to_standard_profile =
		intake_profiler->CreateProfile(STARTING_ANGLE, STANDARD_ANGLE);

const int DOWN_TO_STANDARD_ANGLE = 0;
const int STANDARD_TO_DOWN_ANGLE = 1;
const int STARTING_TO_STANDARD_ANGLE = 2;

int ref_pos_;

int profile_i = 0;

GroundPickup::GroundPickup() {

	canTalonFloorPickupArm = new CANTalon(21);
	canTalonPickupWheel = new CANTalon(22);

	ref_pos_ = STARTING_TO_STANDARD_ANGLE;

}

double GroundPickup::GetVel() {

	return -1.0
			* ((double) canTalonFloorPickupArm->GetEncVel()
					/ (double) ENCODER_TICKS) * NUM_RADS * 10.0;
}

int GroundPickup::GetEncPos() {

	return canTalonFloorPickupArm->GetEncPosition();
}

double GroundPickup::GetPos() {

	return -1.0
			* ((double) canTalonFloorPickupArm->GetEncPosition()
					/ (double) ENCODER_TICKS) * NUM_RADS;

}

void GroundPickup::ZeroPos() {

	canTalonFloorPickupArm->SetEncPosition(0);
}

void GroundPickup::SetPos(double pos) {

	double set_pos = (-1.0 * pos * (double) ENCODER_TICKS) / (NUM_RADS);

	canTalonFloorPickupArm->SetEncPosition(set_pos);

}

void GroundPickup::SpinIn() {

	canTalonPickupWheel->Set(SPIN_SPEED);

}

void GroundPickup::SpinOut() {

	canTalonPickupWheel->Set(-SPIN_SPEED);
}

void GroundPickup::StopSpin() {

	canTalonPickupWheel->Set(0);

}

double GroundPickup::CheckSpinCurrent() {

	return (double) (canTalonPickupWheel->GetOutputCurrent());

}

void GroundPickup::MoveArm(double ref, double profileVelocity) { //position and velocity controller //set profile velocity to 0 if not using motion profiile

	if (ref <= GetPos()) { //going down

		Kp = 0.55;
		Ki = 0.00;
		Kd = 0.00;

		//TODO add some scheduling for the start
	}

	else if (ref > GetPos()) { //going up

		Kp = 4.55;
		Ki = 0.0;
		Kd = 0.0;

	}

//	if (GetPos() >= PI / 2.0) {
//		if (ref >= GetPos()) { //going up
//
//			Kp = 0.5;
//			Ki = 0.00;
//			Kd = 0.00;
//
//		}
//
//		else if (ref < GetPos()) { //going down
//
//			Kp = 4.65;
//			Ki = 0.0;
//			Kd = 0.0;
//
//		}
//	}

	position = GetPos();
	error = ref - position;

	i += error;
	d = error - last_error;

	P = Kp * error;
	I = Ki * i;
	D = Kd * d;

	calculatedFeedForward = ((9.8 * cos(GetPos()) * 2.8 * .25 * 0.0845)
			/ (G * 0.00595)) + (GetVel() / 164.0);

	if ((GetPos() > 1.3 && GetPos() < 1.5)
			|| (GetPos() > 2.0 && GetPos() < 2.2)) { //needs to be even on both sides of the vertical
		calculatedFeedForward = calculatedFeedForward * 1.15;
	} else if (GetPos() >= 1.5 && GetPos() <= 2.0) { //try changing these
		calculatedFeedForward = calculatedFeedForward * 1.25;
	}

	if (GetPos() < (PI / 2.0)) {
		if (ref < .25) {
			calculatedFeedForward = 0.0;
		}
	}

	output = P + I + D + (Kv * profileVelocity) + calculatedFeedForward; //output is in voltage, how it is modeled in matlab

	if (output > MAX_OUTPUT) {
		output = MAX_OUTPUT;
	} else if (output < MIN_OUTPUT) {
		output = MIN_OUTPUT;
	}

	output = output / 12.0; //only works if max output is positive // scaling down -1 to 1

	if (GetPos() > 2.5 && output > 0.0) { //soft limit
		output = 0.0;
	}

	if (GetPos() <= (PI / 2.0)) {
		if (ref <= 0.28) { //.25
			output = 0.0;
		}
	}

	canTalonFloorPickupArm->Set(output);

	last_error = error;

}
void GroundPickup::ClearIAccum() {
	I = 0;
	i = 0;
	last_error = 0;
}
bool GroundPickup::IsAtPosition() {

	if (std::abs(ref_pos_ - GetPos()) <= ACCEPTABLE_ERROR) { //error will be 0 on startup
		return true;
	} else {
		return false;
	}

}

void GroundPickup::GroundPickupStateMachine() { //arm down, spin in, up arm. down arm while spin out. robot drives back, stop spin, up arm

	switch (ground_pickup_state) {

	case arm_up_state:

		ref_pos_ = DOWN_TO_STANDARD_ANGLE;

		StopSpin();

		IsAtPosition();

		break;

	case spin_out_state:

		SpinOut();

		break;

	case spin_in_state:

		SpinIn();

		break;

	case spin_stop_state:

		StopSpin();

		break;

	case arm_down_state:

		StopSpin();

		ref_pos_ = STANDARD_TO_DOWN_ANGLE;

		IsAtPosition();

		break;

	case arm_up_spin_in_state: //gear in

		ref_pos_ = DOWN_TO_STANDARD_ANGLE;

		SpinIn();

		IsAtPosition();

		break;

	case arm_up_spin_out_state:

		ref_pos_ = DOWN_TO_STANDARD_ANGLE;

		SpinOut();

		IsAtPosition();

		break;

	case arm_down_spin_in_state:

		ref_pos_ = STANDARD_TO_DOWN_ANGLE;

		SpinIn();

		IsAtPosition();

		break;

	case arm_down_spin_out_state: //gear out

		ref_pos_ = STANDARD_TO_DOWN_ANGLE;

		SpinOut();

		IsAtPosition();

		break;

	case arm_start_state:

		StopSpin();

		ref_pos_ = STARTING_TO_STANDARD_ANGLE;

		IsAtPosition();

		break;

	case arm_start_spin_in_state:

		ref_pos_ = STARTING_TO_STANDARD_ANGLE;

		SpinIn();

		IsAtPosition();

		break;

	}
}

void GroundPickup::MoveWrapper(GroundPickup *gp, int *ref_pos) {

	std::vector<std::vector<double>> gear_profile = { };

	int gear_index = 0;

	int last_ref = 0;

	int profile = 0;

	timerPickup->Start();

	while (true) { //TODO: need to pass in profile
		while (frc::RobotState::IsEnabled()) {

			std::this_thread::sleep_for(
					std::chrono::milliseconds(PICKUP_SLEEP_TIME));

			if (timerPickup->HasPeriodPassed(PICKUP_WAIT_TIME)) { //if enough time has passed to start a new loop

				//std::cout << "INDEX: " << gp->gearpickup_index << std::endl;
				//std::cout<<"Target: "<< gear_profile.at(0).at(gp->gearpickup_index) <<std::endl;

				profile = *ref_pos;

				switch (profile) {

				case DOWN_TO_STANDARD_ANGLE:
					std::cout << "0" << std::endl;
					gear_profile = down_to_standard_profile;
					if (last_ref != profile) {
						gp->SetIndex(0);
					}
					break;

				case STANDARD_TO_DOWN_ANGLE:
					std::cout << "1" << std::endl;
					gear_profile = standard_to_down_profile;
					if (last_ref != profile) {
						gp->SetIndex(0);
					}
					break;

				case STARTING_TO_STANDARD_ANGLE:
					std::cout << "2" << std::endl;
					gear_profile = starting_to_standard_profile;
					if (last_ref != profile) {
						gp->SetIndex(0);
					}
					break;

				}
				last_ref = profile;

				if (profile == 1) {
					gp->MoveArm(gear_profile.at(0).at(gp->gearpickup_index),
							0.0); //see if all you need to do is not have velocity motion profiles
					std::cout<<"HERE"<<std::endl;
				} else {
					gp->MoveArm(gear_profile.at(0).at(gp->gearpickup_index),
							gear_profile.at(1).at(gp->gearpickup_index));
				}
				std::cout << "Target: "
						<< gear_profile.at(0).at(gp->gearpickup_index)
						<< std::endl;

				//std::cout << "ref" << *ref_pos << std::endl;

				if (gp->gearpickup_index < gear_profile.at(0).size() - 1) {
					gp->gearpickup_index++;
				}

				timerPickup->Reset();

			}

		}
	}

}

void GroundPickup::SetIndex(int index) {
	gearpickup_index = index;
}

int GroundPickup::GetIndex() {
	return gearpickup_index;
}

void GroundPickup::StartThreads() {

	GroundPickup *gp = this;

	PickupThread = std::thread(&GroundPickup::MoveWrapper, gp, &ref_pos_);
	PickupThread.detach();

}

void GroundPickup::DisableThreads() {

	PickupThread.~thread();

}

