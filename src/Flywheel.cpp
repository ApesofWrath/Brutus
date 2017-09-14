/*
 * Flywheel.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <Flywheel.h>

const int stop_state = 0;
const int spin_state = 1;

const int GOAL_RPM_ORIGINAL = 3250;
int GOAL_RPM = 3400;

const int MAX_FLYWHEEL_ERROR = 400; //reaches 3700
//const int CAN_TALON_FLYWHEEL_FRONT_RIGHT = 33;
//const int CAN_TALON_FLYWHEEL_BACK_RIGHT = 24;
//const int CAN_TALON_FLYWHEEL_FRONT_LEFT = 29;
//const int CAN_TALON_FLYWHEEL_BACK_LEFT = 19;
const int CAN_TALON_FLYWHEEL_RIGHT = 0;
const int CAN_TALON_FLYWHEEL_LEFT = 0;
const double FLYWHEEL_WAIT_TIME = 0.01; //ms
const int FLYWHEEL_SLEEP_TIME = 0;

const double RIGHT_F_GAIN = .036; //0.025;
const double RIGHT_P_GAIN = 0.075; //.01;
const double LEFT_F_GAIN = 0.0;
const double LEFT_P_GAIN = 0.0;
const double UNITS_PER_ROT = 4096.0;
const double MINUTE_CONVERSION = 600.0;

bool active_;

double flywheel_time = 0.01; //milliseconds

Timer *timerFly = new Timer();

std::thread SpinThread;

Flywheel::Flywheel() {

	canTalonFlywheelRight = new CANTalon(CAN_TALON_FLYWHEEL_RIGHT);
	canTalonFlywheelRight->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	canTalonFlywheelRight->SetF(RIGHT_F_GAIN);
	canTalonFlywheelRight->SetP(RIGHT_P_GAIN);
	canTalonFlywheelRight->ConfigNominalOutputVoltage(+2.0f, -0.0f);
	canTalonFlywheelRight->ConfigPeakOutputVoltage(+12.0f, +2.0f);
	canTalonFlywheelRight->SetSensorDirection(true); //dont change
	canTalonFlywheelRight->SelectProfileSlot(0);
	canTalonFlywheelRight->SetSensorDirection(false); //false if Koba

	canTalonFlywheelLeft = new CANTalon(CAN_TALON_FLYWHEEL_LEFT);
	canTalonFlywheelLeft->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	canTalonFlywheelLeft->SetF(LEFT_F_GAIN);
	canTalonFlywheelLeft->SetP(LEFT_P_GAIN);
	canTalonFlywheelLeft->ConfigNominalOutputVoltage(+2.0f, -0.0f);
	canTalonFlywheelLeft->ConfigPeakOutputVoltage(+12.0f, +2.0f);
	canTalonFlywheelLeft->SetSensorDirection(true); //dont change
	canTalonFlywheelLeft->SelectProfileSlot(0);
	canTalonFlywheelLeft->SetSensorDirection(false); //false if Koba

	active_ = false;
}

void Flywheel::Spin(int ref) {

	canTalonFlywheelRight->SetControlMode(CANSpeedController::kSpeed);
	canTalonFlywheelRight->Set(ref);

	canTalonFlywheelLeft->SetControlMode(CANSpeedController::kSpeed);
	canTalonFlywheelLeft->Set(ref);

}

void Flywheel::Stop() {

	canTalonFlywheelRight->SetControlMode(CANSpeedController::kVoltage);
	canTalonFlywheelRight->Set(0);

	canTalonFlywheelLeft->SetControlMode(CANSpeedController::kVoltage);
	canTalonFlywheelLeft->Set(0);

}
// current speed target speed variable
bool Flywheel::IsAtSpeedLeft() {

	double flywheel_value = -((double) canTalonFlywheelLeft->GetEncVel()
			/ (double) MINUTE_CONVERSION) * UNITS_PER_ROT;

	if (std::abs(flywheel_value - GOAL_RPM) < MAX_FLYWHEEL_ERROR) {

		return true;

	} else {

		return false;
	}

}

bool Flywheel::IsAtSpeedRight() {

	double flywheel_value = -((double) canTalonFlywheelRight->GetEncVel()
			/ (double) MINUTE_CONVERSION) * UNITS_PER_ROT;

	if (std::abs(flywheel_value - GOAL_RPM) < MAX_FLYWHEEL_ERROR) {

		return true;

	} else {

		return false;
	}

}

void Flywheel::SetGoal(double joyVal) {

	GOAL_RPM = GOAL_RPM_ORIGINAL + (-1 * 200 * joyVal);

}

int Flywheel::GetGoal() {

	return GOAL_RPM;

}

double Flywheel::FlywheelValueRight() {

	double flywheel_value_right = -((double) canTalonFlywheelRight->GetEncVel()
			/ (double) MINUTE_CONVERSION) * UNITS_PER_ROT;

	return flywheel_value_right;

}

double Flywheel::FlywheelValueLeft() {

	double flywheel_value_left = -((double) canTalonFlywheelLeft->GetEncVel()
			/ (double) MINUTE_CONVERSION) * UNITS_PER_ROT;

	return flywheel_value_left;
}

void Flywheel::FlywheelStateMachine() {

	switch (flywheel_state) {

	case stop_state:

		active_ = false;

		Stop();

		break;

	case spin_state:

		active_ = true;

		break;

	}
}

double Flywheel::GetSpeedRight() {

	return -((double) canTalonFlywheelRight->GetEncVel()
			/ (double) MINUTE_CONVERSION) * UNITS_PER_ROT;
}

double Flywheel::GetSpeedLeft() {

	return -((double) canTalonFlywheelLeft->GetEncVel()
			/ (double) MINUTE_CONVERSION) * UNITS_PER_ROT;

}

void Flywheel::SpinWrapper(Flywheel *fw, int ref, bool *active) {

	timerFly->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(FLYWHEEL_SLEEP_TIME));

			while (*active) {

				if (timerFly->HasPeriodPassed(FLYWHEEL_WAIT_TIME)) {

					timerFly->Reset();

					fw->Spin(ref);

				}

			}
		}
	}

}

void Flywheel::StartThread() {

	Flywheel *fw = new Flywheel();

	SpinThread = std::thread(&Flywheel::SpinWrapper, fw, GOAL_RPM, &active_);
	SpinThread.detach();

}
void Flywheel::DisableThreads() {

	SpinThread.~thread();

}
