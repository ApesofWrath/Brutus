/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <TeleopStateMachine.h>
#include <iostream>

const int init_state = 0;
const int wait_for_button_state = 1;
const int open_gear_rails_state = 2;
const int init_shooting_state = 3;
const int fire_state = 4;
const int init_climbing = 5;
const int climbing_state = 6;
const int finish_climbing = 7;
const int gear_pickup_state = 8;
const int gear_score_state = 9;
int state = init_state;

DriveController *drive_Controller;
Flywheel *fly_wheel;
Elevator *elevator_;
GearRail *gear_rail;
Conveyor *conveyor_;
Vision *vision_;
Climber *climber_;
GroundPickup *ground_pickup;

Timer *shootTimer = new Timer();
Timer *drawTimer = new Timer();

const double period = .5;
const double duty_cycle = 1.0;
const double sleep_cycle = 0.0;
const double reverse_cycle = 0.0;

bool elevate_go = false;

double draw_wait_time = 1;

TeleopStateMachine::TeleopStateMachine(Flywheel *flywheelP, Conveyor *conveyorP,
		GearRail *gearRailP, Elevator *elevatorP,
		DriveController *driveControllerP, Vision *visionP, Climber *climberP, GroundPickup *groundPickupP) {

	fly_wheel = flywheelP;

	conveyor_ = conveyorP;

	gear_rail = gearRailP;

	elevator_ = elevatorP;

	drive_Controller = driveControllerP;

	vision_ = visionP;

	climber_ = climberP;

	ground_pickup = groundPickupP;

}

void TeleopStateMachine::StateMachine(bool is_gear, bool is_close_gear,
bool is_fire, bool is_climb,
bool return_button, bool is_popcorn, bool is_second_fire, bool is_stop_shoot, bool gear_pickup_button, bool gear_score_button, bool is_intake, bool is_outtake, bool is_arm_up, bool is_arm_down) {

	if (fly_wheel->IsAtSpeedRight() && fly_wheel->IsAtSpeedLeft()) {

		SmartDashboard::PutBoolean("At Speed", true);

	} else {

		SmartDashboard::PutBoolean("At Speed", false);

	}

	SmartDashboard::PutNumber("Right FlyWheel Speed", fly_wheel->GetSpeedRight());
	SmartDashboard::PutNumber("Left FlyWheel Speed", fly_wheel->GetSpeedLeft());

	if (is_popcorn) {

		elevator_->elevator_state = elevator_->reverse_state_h;

	} else if (!is_popcorn && state != fire_state) {

		elevator_->elevator_state = elevator_->stop_state_h;

	}

	if (is_gear) { // gear rails code - not used anymore

		gear_rail->gear_rail_state = gear_rail->open_state_h;

	} else if (is_close_gear) {

		gear_rail->gear_rail_state = gear_rail->close_state_h;
	}

	if (is_intake) {

		ground_pickup->ground_pickup_state = ground_pickup->spin_in_state_h;

	}

	if (is_outtake) {

		ground_pickup->ground_pickup_state = ground_pickup->spin_out_state_h;
	}

	if (is_arm_up) {

		ground_pickup->ground_pickup_state = ground_pickup->arm_up_state_h;

	}

	if (is_arm_down) {

		ground_pickup->ground_pickup_state = ground_pickup->arm_down_state_h;

	}

	//START SWITCH
	switch (state) {

	case init_state:

		SmartDashboard::PutString("State", "Initial");

		gear_rail->gear_rail_state = gear_rail->close_state_h;

		fly_wheel->flywheel_state = fly_wheel->stop_state_h;

		conveyor_->conveyor_state = conveyor_->stop_state_h;

		elevator_->elevator_state = elevator_->stop_state_h;

		ground_pickup->ground_pickup_state = ground_pickup->arm_up_state_h;

		state = wait_for_button_state;

		break;

	case wait_for_button_state:

		SmartDashboard::PutString("State", "Wait For Button");

		fly_wheel->flywheel_state = fly_wheel->stop_state_h;

		climber_->climber_state = climber_->stop_state_h;

		if (is_fire) {

			state = init_shooting_state;

		} else if (is_climb) {

			state = init_climbing;

		}

		if(gear_pickup_button) { //cannot pick up gear while doing other things

			state = gear_pickup_state;
		}

		if(gear_score_button) {

			state = gear_score_state;

		}


		break;

	case init_shooting_state:

		SmartDashboard::PutString("State", "Initial Shooting");

		fly_wheel->flywheel_state = fly_wheel->spin_state_h;

		if (is_stop_shoot) {

			state = wait_for_button_state;

		}

		if (fly_wheel->IsAtSpeedLeft() && fly_wheel->IsAtSpeedRight() && is_second_fire) {

			shootTimer->Reset();

			shootTimer->Start();

			state = fire_state;
		}

		break;

	case fire_state:

		SmartDashboard::PutString("State", "Fire");

		if (is_popcorn) {

			elevator_->elevator_state = elevator_->reverse_state_h;

		} else if ((shootTimer->Get() < (duty_cycle * period))) {

			elevator_->elevator_state = elevator_->elevate_state_h;

		}else if ((shootTimer->Get() < ((reverse_cycle * period)) + (duty_cycle * period))){

			elevator_->elevator_state = elevator_->reverse_state_h;

		}else if ((shootTimer->Get() < ((reverse_cycle * period)) + (duty_cycle * period) + (sleep_cycle * period))){

			elevator_->elevator_state = elevator_->stop_state_h;

		}else if ((shootTimer->Get() >= ((reverse_cycle * period)) + (duty_cycle * period) + (sleep_cycle * period)) && (shootTimer->Get() < (period))) { //stop during other time

			elevator_->elevator_state = elevator_->stop_state_h;

		}else {//if ((shootTimer->Get() >= (period))) { // automatically resets the timer

			shootTimer->Reset();

		}

		if (is_stop_shoot) {

			state = wait_for_button_state;

			elevator_->elevator_state = elevator_->stop_state_h;

			shootTimer->Stop();

		} else if (!is_second_fire) { //must hold button to keep spinning

			elevator_->elevator_state = elevator_->stop_state_h;

			shootTimer->Stop();

			state = init_shooting_state;

		}

		break;

	case init_climbing:

		SmartDashboard::PutString("State", "Initial Climbing");

		elevator_->elevator_state = elevator_->stop_state_h;

		fly_wheel->flywheel_state = fly_wheel->stop_state_h;

		state = climbing_state;

		break;

	case climbing_state:

		SmartDashboard::PutString("State", "Climbing");

		climber_->climber_state = climber_->climbing_state_h;

		if (!is_climb) {//hold to climb, release to stop

			state = wait_for_button_state;
		}

		if (climber_->CheckCurrent() >= climber_->MAX_CURRENT) {

			drawTimer->Start();

			if (drawTimer->HasPeriodPassed(draw_wait_time)){

				state = wait_for_button_state;

			}
		}

		break;

	case finish_climbing: //not used

		SmartDashboard::PutString("State", "Finish Climbing");

		climber_->climber_state = climber_->stop_state_h;

		if (return_button) {

			state = wait_for_button_state;

		}

		break;

	case gear_pickup_state:

		SmartDashboard::PutString("State", "Gear In");

		if(return_button) {

			state = wait_for_button_state;

		}

		ground_pickup->ground_pickup_state = ground_pickup->arm_down_spin_in_state_h;

		if (ground_pickup->CheckSpinCurrent() >= ground_pickup->MAX_SPIN_CURRENT) { //gear is in

			ground_pickup->ground_pickup_state = ground_pickup->arm_up_state_h;

			if (ground_pickup->IsAtPosition()) {

				state  = wait_for_button_state;

			}
		}

		break;

	case gear_score_state:

		SmartDashboard::PutString("State", "Gear Out");

		if(return_button) {

			state = wait_for_button_state;

		}

		ground_pickup->ground_pickup_state = ground_pickup->arm_down_spin_out_state_h;

		if(ground_pickup->IsAtPosition()) {

			ground_pickup->ground_pickup_state = ground_pickup->arm_down_state_h; //no spin
			state = wait_for_button_state;

		}
		break;

	}
//END SWITCH

}


void TeleopStateMachine::Initialize() {

	state = init_state;

}

