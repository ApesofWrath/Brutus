/*
 * DriveController.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: Apes of Wrath
 */

#include <DriveController.h>
#include <WPILib.h>

#define PI 3.1415926

using namespace std::chrono;

const double WHEEL_DIAMETER = 4.0; //inches
const double TICKS_PER_ROT = 4096.0;

const double MAX_Y_RPM = 625;
double DYN_MAX_Y_RPM = 625;
const double MAX_X_RPM = 400; // Max RPM ACTUAL: 330
const double MAX_YAW_RATE = (19.04 / 625) * MAX_Y_RPM; //max angular velocity divided by the max rpm multiplied by set max rpm

const int DRIVE_SLEEP_TIME = 0.00;
const double DRIVE_WAIT_TIME = 0.01; //10 ms

const int CAN_TALON_FRONT_LEFT = 18; //22 is gear pickup
const int CAN_TALON_BACK_LEFT = 32;
const int CAN_TALON_BACK_RIGHT = 30;
const int CAN_TALON_FRONT_RIGHT = 36;
const int CAN_TALON_KICKER = 20;

double l_last_error = 0;
double r_last_error = 0;
double yaw_last_error = 0;
double kick_last_error = 0;

double l_last_error_vel = 0;
double r_last_error_vel = 0;
double kick_last_error_vel = 0;

//CHANGEABLESTART

const double K_P_YAW_T = 0.0; //4 //joystick now cubed

const double K_P_YAW_AU = 0.0; //10
const double K_D_YAW_AU = 0.0;

const double K_P_YAW_H_VEL = 13.0;

const double K_P_YAW_HEADING_POS = 9.0;

const double K_D_VISION_POS = 0.0;

const double K_P_LEFT_VEL = 0.0; //0.004220; //0.004
const double K_D_LEFT_VEL = 0.0;
const double K_F_LEFT_VEL = 1.0 / 625.0;
double P_LEFT_VEL = 0;
double D_LEFT_VEL = 0;
double d_left_vel = 0; //dynamic value

const double K_P_RIGHT_VEL = 0.0; //0.004220; //0.004
const double K_D_RIGHT_VEL = 0.0;
const double K_F_RIGHT_VEL = 1.0 / 625.0;
double P_RIGHT_VEL = 0;
double D_RIGHT_VEL = 0;
double d_right_vel = 0; //dynamic value

const double K_P_KICK_VEL = 0.0; //.00365; //0.00311
const double K_D_KICK_VEL = 0.0;
const double K_F_KICK_VEL = 1.0 / 400.0;
double P_KICK_VEL = 0;
double D_KICK_VEL = 0;
double d_kick_vel = 0; // dynamic value

const double CONVERSION_DIVISION = 4096;
const double CONVERSION_MULTIPLICATION = 600;

const double K_P_RIGHT_DIS = 0.088; //0.225
const double K_P_LEFT_DIS = 0.088; //0.225
const double K_P_KICKER_DIS = 0.280; //TODO: check this value
const double K_P_YAW_DIS = 0.558;

const double K_I_RIGHT_DIS = 0.055;
const double K_I_LEFT_DIS = 0.055;
const double K_I_KICKER_DIS = 0.0;
const double K_I_YAW_DIS = 0.11;

const double K_D_RIGHT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0;

//CHANGEABLE END

const double MAX_FPS = ((MAX_Y_RPM * 4.0 * PI) / 12.0) / 60.0; //conversion to fps
const double Kv = 1 / MAX_FPS; //scale from -1 to 1

const double MAX_KICK_FPS = ((MAX_X_RPM * 4.0 * PI) / 12.0) / 60.0;
const int Kv_KICK = 1 / MAX_KICK_FPS;

double P_RIGHT_DIS = 0;
double I_RIGHT_DIS = 0;
double D_RIGHT_DIS = 0;

double P_LEFT_DIS = 0;
double I_LEFT_DIS = 0;
double D_LEFT_DIS = 0;

double P_KICK_DIS = 0;
double I_KICK_DIS = 0;
double D_KICK_DIS = 0;

double P_YAW_DIS = 0;
double I_YAW_DIS = 0;

double d_vision = 0;

double d_right = 0;
double i_right = 0;

double d_yaw_dis = 0;

double d_left = 0;
double i_left = 0;

double d_kick = 0;
double i_kick = 0;

double i_yaw = 0;

double l_error_vel_au = 0;
double l_error_dis_au = 0;

double r_error_vel_au = 0;
double r_error_dis_au = 0;

double k_error_dis_au = 0;

double y_error_dis_au = 0; // yaw (theta) position error

double l_error_vel_t = 0;
double l_error_dis_t = 0;

double r_error_vel_t = 0;
double r_error_dis_t = 0;

double kick_error_vel = 0;

double drive_wait_time = 0.01; //ms

const int NUM_POINTS = 1500;
const int NUM_INDEX = 15;

//double last_drive_ref[NUM_INDEX] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 0.0};
double drive_ref[NUM_INDEX] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double full_refs[NUM_POINTS][NUM_INDEX];

double acceptable_yaw_error = .22;

double error_heading = 0;
double last_error_heading = 0;
double total_heading = 0;

double init_heading = 0;
double visionAngle = 0;
double visionDistance = 0;

double store_right_enc = 0;
double store_left_enc = 0;

const int stamp_state = 0;
const int vision_aim_state = 1;
const int drive_state = 2;

bool vision_done_before = false;

int vision_track_state = stamp_state;

Timer *timerTeleop = new Timer();
Timer *timerAuton = new Timer();
Timer *observer = new Timer();

std::thread HDriveThread, DrivePIDThread;

int profile_index = 0;

int *ptr_index = &profile_index; //setting pointer to index

Vision *vision_dc = new Vision();

int vision_index = 0;
bool insert_profile = true;

std::vector<std::vector<double>> vision_profile = { };

TrapezoidalProfile *vision_profiler = new TrapezoidalProfile(15, 5, .01);

DriveController::DriveController(Vision *vis) { //front talon id's are set to the back motors (opposite the gear pickup) for encoder reasons

	vision_dc = vis;

	canTalonFrontLeft = new CANTalon(CAN_TALON_FRONT_LEFT);

	canTalonBackLeft = new CANTalon(CAN_TALON_BACK_LEFT);
//	canTalonBackLeft->SetControlMode(CANSpeedController::kFollower);
//	canTalonBackLeft->Set(CAN_TALON_FRONT_LEFT);

	canTalonFrontRight = new CANTalon(CAN_TALON_FRONT_RIGHT);

	canTalonBackRight = new CANTalon(CAN_TALON_BACK_RIGHT);
//	canTalonBackRight->SetControlMode(CANSpeedController::kFollower);
//	canTalonBackRight->Set(CAN_TALON_FRONT_RIGHT);

	canTalonKicker = new CANTalon(CAN_TALON_KICKER);

	ahrs = new AHRS(SPI::Port::kMXP, 200);

	kickerPiston = new DoubleSolenoid(4, 5, 6);

}

void DriveController::HDrive(Joystick *JoyThrottle, Joystick *JoyWheel,
bool *is_fc) { //finds targets for teleop

	double forward = -1.0 * (JoyThrottle->GetY());
	double strafe = (JoyThrottle->GetX());
	double current_yaw = (fmod((-1.0 * ahrs->GetYaw() * (PI / 180.0)),
			(2.0 * PI)))
			* (fmod((-1.0 * ahrs->GetYaw() * (PI / 180.0)), (2.0 * PI))); //even less sensitive

	if ((bool) *is_fc) {

		if (current_yaw < -PI) {
			current_yaw += (2.0 * PI);
		} else if (current_yaw > PI) {
			current_yaw -= (2.0 * PI);
		}

		double psi = std::atan2(forward, strafe);

		double magnitude = sqrt((forward) + (strafe));

		forward = magnitude * (std::sin(psi - current_yaw));
		strafe = magnitude * (std::cos(psi - current_yaw));

		if ((psi - current_yaw) > (-1.0 * PI) && (psi - current_yaw) <= (0)) {
			if (forward > 0) {
				forward = -1.0 * forward;
			}
		}
		if (((psi - current_yaw) > (PI / 2) && (psi - current_yaw) <= (PI))
				|| ((psi - current_yaw) > (-1.0 * PI)
						&& (psi - current_yaw) <= (-1.0 * PI / 2))) {
			if (strafe > 0) {
				strafe = -1.0 * strafe;
			}
		}
	} else {

		forward = 1.0 * forward; //not needed

	}

	double target_l, target_r, target_kick, target_yaw_rate;

	double axis_ratio = 0.0; //ratio between x and y axes

	if (strafe != 0) { //if x is 0, it is undefined (division)
		axis_ratio = std::abs(forward / strafe); //dont use regular abs, that returns an int
		DYN_MAX_Y_RPM = MAX_X_RPM * (double) axis_ratio;
		DYN_MAX_Y_RPM = DYN_MAX_Y_RPM > MAX_Y_RPM ? MAX_Y_RPM : DYN_MAX_Y_RPM; //if DYN_max_Y is bigger than MAX_Y then set to MAX_Y, otherwise keep DYN_MAX_Y
	} else {
		DYN_MAX_Y_RPM = MAX_Y_RPM; //deals with special case tht x = 0 (ratio cant divide by zero)
	}

	target_l = 1.0 * (forward < 0 ? -1 : 1) * (forward * forward)
			* DYN_MAX_Y_RPM; //if joy value is less than 0 set to -1, otherwise to 1

	target_r = target_l;

	target_kick = 1.0 * (strafe < 0 ? -1 : 1) * (strafe * strafe) * MAX_X_RPM; //if joy value is less than 0 set to -1, otherwise to 1

	double joy_wheel_val = JoyWheel->GetX();

	if (std::abs(joy_wheel_val) < .02) {
		joy_wheel_val = 0.0;
	}

	target_yaw_rate = -1.0 * (joy_wheel_val) * MAX_YAW_RATE; //Left will be positive

	if (abs(target_kick) < 35) {
		target_kick = 0;
	}

	if (target_l > MAX_Y_RPM) {
		target_l = MAX_Y_RPM;
	} else if (target_l < -MAX_Y_RPM) {
		target_l = -MAX_Y_RPM;
	}

	if (target_r > MAX_Y_RPM) {
		target_r = MAX_Y_RPM;
	} else if (target_r < -MAX_Y_RPM) {
		target_r = -MAX_Y_RPM;
	}

	Drive(target_kick, target_r, target_l, target_yaw_rate, K_P_RIGHT_VEL,
			K_P_LEFT_VEL, K_P_KICK_VEL, K_P_YAW_T, 0.0, K_D_LEFT_VEL,
			K_D_RIGHT_VEL, K_D_KICK_VEL, 0.0, 0.0, 0.0);

}

/**
 * Param: Feet forward, + = forward
 */
void DriveController::DrivePID() { //auton

	double refYaw = drive_ref[0];
	double refLeft = drive_ref[1];
	double refRight = drive_ref[2];
	double refKick = drive_ref[3];
	double targetYawRate = drive_ref[4];
	double tarVelLeft = drive_ref[5];
	double tarVelRight = drive_ref[6];
	double tarVelKick = drive_ref[7];

	if (std::abs(tarVelKick) < .05) {
		tarVelKick = 0.0;
	}

	double r_current = -((double) canTalonFrontRight->GetEncVel()
			/ (double) CONVERSION_DIVISION) * CONVERSION_MULTIPLICATION;
	double l_current = ((double) canTalonFrontLeft->GetEncVel()
			/ (double) CONVERSION_DIVISION) * CONVERSION_MULTIPLICATION;

	//conversion to feet
	double r_dis = -(((double) canTalonFrontRight->GetEncPosition()
			/ TICKS_PER_ROT) * (WHEEL_DIAMETER * PI) / 12);
	double l_dis = (((double) canTalonFrontLeft->GetEncPosition()
			/ TICKS_PER_ROT) * (WHEEL_DIAMETER * PI) / 12);
	double k_dis = (((double) canTalonKicker->GetEncPosition() / TICKS_PER_ROT)
			* (WHEEL_DIAMETER * PI) / 12);
	double y_dis = -1.0 * ahrs->GetYaw() * (double) (PI / 180); //current theta (yaw) value

	l_error_dis_au = refLeft - l_dis;
	r_error_dis_au = refRight - r_dis;
	k_error_dis_au = refKick - k_dis;
	y_error_dis_au = refYaw - y_dis;

	if (std::abs(tarVelLeft - tarVelRight) < .05 && (std::abs(r_current) < 10)
			&& (std::abs(l_current) < 10)) {

		y_error_dis_au = 0;

	}

	//std::cout << "Error: " << y_error_dis_au << std::endl;

	i_right += (r_error_dis_au);
	d_right = (r_error_dis_au - r_last_error);

	i_left += (l_error_dis_au);
	d_left = (l_error_dis_au - l_last_error);

	i_kick += (k_error_dis_au);
	d_kick = (k_error_dis_au - kick_last_error);

	i_yaw += y_error_dis_au;

	P_RIGHT_DIS = K_P_RIGHT_DIS * r_error_dis_au;
	I_RIGHT_DIS = K_I_RIGHT_DIS * i_right;
	D_RIGHT_DIS = K_D_RIGHT_DIS * d_right;

	P_LEFT_DIS = K_P_LEFT_DIS * l_error_dis_au;
	I_LEFT_DIS = K_I_LEFT_DIS * i_left;
	D_LEFT_DIS = K_D_LEFT_DIS * d_left;

	P_KICK_DIS = K_P_KICKER_DIS * k_error_dis_au;
	I_KICK_DIS = K_I_KICKER_DIS * i_kick;
	D_KICK_DIS = K_D_KICKER_DIS * d_kick;

	P_YAW_DIS = K_P_YAW_DIS * y_error_dis_au;
	I_YAW_DIS = K_I_YAW_DIS * i_yaw;

	double total_right = P_RIGHT_DIS + I_RIGHT_DIS + D_RIGHT_DIS;
	double total_left = P_LEFT_DIS + I_LEFT_DIS + D_LEFT_DIS;
	double total_kick = P_KICK_DIS + I_KICK_DIS + D_KICK_DIS;
	double total_yaw = P_YAW_DIS + I_YAW_DIS;

	double target_rpm_yaw_change = total_yaw * MAX_Y_RPM;
	double target_rpm_right = total_right * MAX_Y_RPM;
	double target_rpm_left = total_left * MAX_Y_RPM;
	double target_rpm_kick = total_kick * MAX_X_RPM;

	target_rpm_right = target_rpm_right + target_rpm_yaw_change;
	target_rpm_left = target_rpm_left - target_rpm_yaw_change;

	if (target_rpm_left > MAX_Y_RPM) {
		target_rpm_left = MAX_Y_RPM;
	} else if (target_rpm_left < -MAX_Y_RPM) {
		target_rpm_left = -MAX_Y_RPM;
	}

	if (target_rpm_right > MAX_Y_RPM) {
		target_rpm_right = MAX_Y_RPM;
	} else if (target_rpm_right < -MAX_Y_RPM) {
		target_rpm_right = -MAX_Y_RPM;
	}

	Drive(target_rpm_kick, target_rpm_right, target_rpm_left, targetYawRate,
			K_P_RIGHT_VEL, K_P_LEFT_VEL, K_P_KICK_VEL, K_P_YAW_AU, K_D_YAW_AU,
			K_D_LEFT_VEL, K_D_RIGHT_VEL, K_D_KICK_VEL, tarVelLeft, tarVelRight,
			tarVelKick);

	l_last_error = l_error_dis_au;
	r_last_error = r_error_dis_au;
	kick_last_error = k_error_dis_au;

	//std::cout << "L: " << target_rpm_left << std::endl;
	//std::cout << " L: " << P_LEFT_DIS << std::endl;

}

/*
 * Param: radian value, + is right
 */
void DriveController::HeadingPID(Joystick *joyWheel) { //angling

	double target_heading = init_heading
			+ (-1.0 * joyWheel->GetX() * (90.0 * PI / 180.0)); //scaling, conversion to radians,left should be positive

	double current_heading = -1.0 * ahrs->GetYaw() * ( PI / 180.0); //degrees to radians, left should be positive

	double error_heading_h = target_heading - current_heading;

	double total_heading_h = K_P_YAW_HEADING_POS * error_heading_h;

	if (total_heading > MAX_YAW_RATE) {
		total_heading = MAX_YAW_RATE;
	} else if (total_heading < -MAX_YAW_RATE) {
		total_heading = -MAX_YAW_RATE;
	}

	Drive(0.0, 0.0, 0.0, total_heading_h, K_P_RIGHT_VEL, K_P_LEFT_VEL,
			K_P_KICK_VEL, K_P_YAW_H_VEL, 0.0, K_D_RIGHT_VEL, K_D_LEFT_VEL,
			K_D_KICK_VEL, 0.0, 0.0, 0.0);

}

void DriveController::VisionP(double ref) { //auto-aiming

	double angle = ref; //visionAngle set in robot.cpp in the switch statement

	double normalized_angle = 0;

	if (angle > 180) {
		normalized_angle = 360 - angle;
	} else {
		normalized_angle = -angle; //negative = right
	}

	normalized_angle = normalized_angle * (double) (PI / 180.0);

	double current_heading = -1.0 * ahrs->GetYaw() * (double) (PI / 180.0);

	error_heading = (init_heading + normalized_angle) - current_heading;

	d_vision = (error_heading - last_error_heading);

	total_heading = (K_P_YAW_HEADING_POS * error_heading)
			+ (d_vision * K_D_VISION_POS);

	if (total_heading > MAX_YAW_RATE) {
		total_heading = MAX_YAW_RATE;
	} else if (total_heading < -MAX_YAW_RATE) {
		total_heading = -MAX_YAW_RATE;
	}

	//call to velocit controller
	Drive(0.0, 0.0, 0.0, total_heading, K_P_RIGHT_VEL, K_P_LEFT_VEL,
			K_P_KICK_VEL, K_P_YAW_H_VEL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

}

void DriveController::AutoVisionTrack() { //aims the robot and then drive forward in auton, do not use n teleop

	switch (vision_track_state) {

	case stamp_state:
		SetInitHeading();
		SetAngle();
		vision_track_state = vision_aim_state;
		break;

	case vision_aim_state:
		VisionP(visionAngle); //teleop
		if (!vision_done_before && (total_heading < 0.4)) { //TODO: MAGYK NUBER
			vision_track_state = stamp_state;
			vision_done_before = true;
		}
		if (vision_done_before && (std::abs(error_heading) < 2.0)) { //TODO: MAGYK NUBER
			ZeroEncs();
			SetDist();
			vision_track_state = drive_state;
		}
		break;

	case drive_state:
		drive_ref[0] = visionDistance; //auton
		drive_ref[1] = visionDistance;
		drive_ref[2] = 0;
		drive_ref[3] = 0;
		DrivePID();
		break;

	}

}

void DriveController::Drive(double ref_kick, double ref_right,
		double ref_left, //teleop
		double ref_yaw, double k_p_right, double k_p_left, double k_p_kick,
		double k_p_yaw, double k_d_yaw, double k_d_right, double k_d_left,
		double k_d_kick, double target_vel_left, double target_vel_right,
		double target_vel_kick) {

	double yaw_rate_current = -1.0 * (double) ahrs->GetRawGyroZ()
			* (double) ((PI) / 180); //left should be positive

	double target_yaw_rate = ref_yaw;

	ref_left = ref_left - (target_yaw_rate * (MAX_Y_RPM / MAX_YAW_RATE)); //left should be positive
	ref_right = ref_right + (target_yaw_rate * (MAX_Y_RPM / MAX_YAW_RATE));

	double yaw_error = target_yaw_rate - yaw_rate_current;

	if (std::abs(yaw_error) < .25) {
		yaw_error = 0;
	}

	d_yaw_dis = yaw_error - yaw_last_error;

	double yaw_output = ((k_p_yaw * yaw_error) + (k_d_yaw * d_yaw_dis));

	ref_right += yaw_output; //left should be positive
	ref_left -= yaw_output;

	if (std::abs(ref_kick) < 25) {
		ref_kick = 0;
	}

	if (ref_left > MAX_Y_RPM) {
		ref_left = MAX_Y_RPM;
	} else if (ref_left < -MAX_Y_RPM) {
		ref_left = -MAX_Y_RPM;
	}

	if (ref_right > MAX_Y_RPM) {
		ref_right = MAX_Y_RPM;
	} else if (ref_right < -MAX_Y_RPM) {
		ref_right = -MAX_Y_RPM;
	}

	double feed_forward_r = K_F_RIGHT_VEL * ref_right;
	double feed_forward_l = K_F_LEFT_VEL * ref_left;
	double feed_forward_k = K_F_KICK_VEL * ref_kick;

	//conversion to RPM from native unit
	double l_current = ((double) canTalonFrontLeft->GetEncVel()
			/ (double) CONVERSION_DIVISION) * CONVERSION_MULTIPLICATION;
	double r_current = -((double) canTalonFrontRight->GetEncVel()
			/ (double) CONVERSION_DIVISION) * CONVERSION_MULTIPLICATION;
	double kick_current = ((double) canTalonKicker->GetEncVel()
			/ (double) CONVERSION_DIVISION) * CONVERSION_MULTIPLICATION; //going right is positive

	l_error_vel_t = ref_left - l_current;
	r_error_vel_t = ref_right - r_current;
	kick_error_vel = ref_kick - kick_current;

	d_left_vel = (l_error_vel_t - l_last_error_vel);
	d_right_vel = (r_error_vel_t - r_last_error_vel);
	d_kick_vel = (kick_error_vel - kick_last_error_vel);

	P_LEFT_VEL = k_p_left * l_error_vel_t;
	P_RIGHT_VEL = k_p_right * r_error_vel_t;
	P_KICK_VEL = k_p_kick * kick_error_vel;

	D_LEFT_VEL = k_d_left * d_left_vel;
	D_RIGHT_VEL = k_d_right * d_right_vel;
	D_KICK_VEL = k_d_kick * d_kick_vel;

	//if (frc::RobotState::IsAutonomous()){ //only want the feedforward based off the motion profile during autonomous. The root generated ones (in the if() statement)
	//	feed_forward_r = 0;				  // will be close to 0  (low error between profile points) for the most part but will get quite aggressive when an error builds,
	//	feed_forward_l = 0;				  //the PD controller should handle it itself
	//	feed_forward_k = 0;
	//}

	double total_right = D_RIGHT_VEL + P_RIGHT_VEL + feed_forward_r
			+ (Kv * target_vel_right);
	double total_left = D_LEFT_VEL + P_LEFT_VEL + feed_forward_l
			+ (Kv * target_vel_left);
	double total_kick = D_KICK_VEL + P_KICK_VEL + feed_forward_k
			+ (Kv_KICK * target_vel_kick);

	canTalonFrontLeft->Set(-total_left); //back cantalons follow front, don't need to set them individually
	canTalonFrontRight->Set(total_right);
	canTalonBackRight->Set(total_right);
	canTalonBackLeft->Set(-total_left);
	canTalonKicker->Set(-total_kick);

//	std::cout << "P: " << P_LEFT_VEL;
//	std::cout << " Ref: " << ref_kick;
//	std::cout << " Left: " << l_current;
//	std::cout << " Right: " << r_current;
	//std::cout << " Error: " << kick_error_vel << std::endl;
//	std::cout << "YAW RATE: " << yaw_rate_current;
//	std::cout << " ERROR: " << yaw_error << std::endl;

//	std::cout << "R: " << r_current;<< " L: " << l_current << std::endl;

	yaw_last_error = yaw_error;
	l_last_error_vel = l_error_vel_t;
	r_last_error_vel = r_error_vel_t;
	kick_last_error_vel = kick_error_vel;

}

void DriveController::ResetVisionState() {

	vision_track_state = stamp_state;

	vision_done_before = false;

}

void DriveController::StopAll() {

	canTalonFrontLeft->Set(0);

	canTalonFrontRight->Set(0);

	canTalonBackLeft->Set(0);

	canTalonBackRight->Set(0);

	canTalonKicker->Set(0);
}

void DriveController::KickerUp() {

	kickerPiston->Set(DoubleSolenoid::Value::kReverse);

}

void DriveController::KickerDown() {

	kickerPiston->Set(DoubleSolenoid::Value::kForward);
}

void DriveController::ZeroEncs() {

	canTalonFrontRight->SetEncPosition(0);
	canTalonFrontLeft->SetEncPosition(0);
	canTalonBackRight->SetEncPosition(0);
	canTalonBackLeft->SetEncPosition(0);
	canTalonKicker->SetEncPosition(0);

}

void DriveController::ZeroI() {

	i_right = 0;
	i_left = 0;
	i_yaw = 0;

	y_error_dis_au = 0;
	l_error_dis_au = 0;
	r_error_dis_au = 0;

	StopAll();

}

void DriveController::SetInitHeading() {

	init_heading = -1.0 * ahrs->GetYaw() * (double) (PI / 180);

}

void DriveController::SetAngle() {

	visionAngle = vision_dc->findAzimuth(); //returns in degrees

}

void DriveController::SetDist() {

	visionDistance = vision_dc->findDistance() / 12.0; //convert to feet

}

bool DriveController::CheckIfNull() { //true if all values are 0

	for (int i = 0; i < NUM_INDEX; i++) { //checks if the entire array is zero (excluding the first point, make sure you dont check that one)
		if (drive_ref[i] != 0) {
			return false;
		}
	}
	return true;

}

void DriveController::StoreEncValues() {

	store_right_enc = -canTalonFrontRight->GetEncPosition();
	store_left_enc = canTalonFrontLeft->GetEncPosition();

}

void DriveController::SetEncValues() {

	canTalonFrontRight->SetEncPosition(store_right_enc);
	canTalonBackRight->SetEncPosition(store_right_enc);

	canTalonFrontLeft->SetEncPosition(store_left_enc);
	canTalonBackLeft->SetEncPosition(store_left_enc);

}

void DriveController::HDriveWrapper(Joystick *JoyThrottle, Joystick *JoyWheel,
bool *is_heading, bool *is_vision, bool *is_fc,
		DriveController *driveController) {

	timerTeleop->Start();

	while (true) {
		while (frc::RobotState::IsEnabled() && !frc::RobotState::IsAutonomous()
				&& !(bool) *is_heading && !(bool) *is_vision) {

			std::this_thread::sleep_for(
					std::chrono::milliseconds(DRIVE_SLEEP_TIME));

			if (timerTeleop->HasPeriodPassed(DRIVE_WAIT_TIME)) {

				driveController->HDrive(JoyThrottle, JoyWheel, is_fc);

				timerTeleop->Reset();

				//	std::cout<<"here"<< std::endl;

			}

		}
		while (frc::RobotState::IsEnabled() && !frc::RobotState::IsAutonomous()
				&& (bool) *is_heading && !(bool) *is_vision) {

			std::this_thread::sleep_for(
					std::chrono::milliseconds(DRIVE_SLEEP_TIME));

			if (timerTeleop->HasPeriodPassed(DRIVE_WAIT_TIME)) {

				driveController->HeadingPID(JoyWheel);

				timerTeleop->Reset();

			}
		}
		while (frc::RobotState::IsEnabled() && !frc::RobotState::IsAutonomous()
				&& (bool) *is_vision && !(bool) *is_heading) {

//			if (insert_profile == true) { // checks if the loop (vision tracking) is running for the first time, will run only once: every time the tracking button is held.
//
//				vision_profile = vision_profiler->CreateProfile(0, (visionAngle * (PI / 180.0))); //need to pass through radians as the controller calculates in radians
//													   // VisionP expects targets not added to the current position: angle to target only
//			}
//
//			insert_profile = false;

			std::this_thread::sleep_for(
					std::chrono::milliseconds(DRIVE_SLEEP_TIME));

			if (timerTeleop->HasPeriodPassed(DRIVE_WAIT_TIME)) {

//				//driveController->VisionP(vision_profile.at(0).at(vision_index));
//
//				if (vision_index < (sizeof(vision_profile.at(0)) - 1)) {
//					vision_index++;
//				}
//
				timerTeleop->Reset();

			}

		}

		vision_index = 0;
		insert_profile = true;
	}
}

void DriveController::DrivePIDWrapper(DriveController *driveController) { //auton

	timerAuton->Start();

	while (frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled()) {

		std::this_thread::sleep_for(
				std::chrono::milliseconds(DRIVE_SLEEP_TIME));

		if (timerAuton->HasPeriodPassed(DRIVE_WAIT_TIME)) {

			for (int i = 0; i < sizeof(drive_ref); i++) { //looks through each row and then fills drive_ref with the column here, refills each interval with next set of refs

				drive_ref[i] = full_refs[profile_index][i]; //from SetRef()

			}

			if (drive_ref[12] == 1) { //vision on

				driveController->AutoVisionTrack();

			} else { //vision off

				driveController->DrivePID();

			}

			timerAuton->Reset();

			profile_index++;
		}

		if (profile_index >= NUM_POINTS) { //stop at the end of the motion profile, this number is set after the creation of the array
			//so not all of the array will be accessed, only the part before the non-zero points
			break;
		}
	}

}

void DriveController::StartTeleopThreads(Joystick *JoyThrottle, //must pass in parameters to wrapper to use them in functions
		Joystick *JoyWheel,
		bool *is_heading, bool *is_vision, bool *is_fc) {

	DriveController *dc = this;

	HDriveThread = std::thread(&DriveController::HDriveWrapper, JoyThrottle,
			JoyWheel, is_heading, is_vision, is_fc, dc);
	HDriveThread.detach();

}

void DriveController::StartAutonThreads() {

	DriveController *dc = this;

	DrivePIDThread = std::thread(&DriveController::DrivePIDWrapper, dc);
	DrivePIDThread.detach();

}

void DriveController::DisableTeleopThreads() {

	HDriveThread.~thread();

}

void DriveController::DisableAutonThreads() {

	DrivePIDThread.~thread();

}

void DriveController::SetRef(double ref[][15]) { //each point has 15 indexes, last column is 0s

	for (int r = 0; r < (sizeof(full_refs) / sizeof(full_refs[0])); r++) {

		for (int c = 0; c < (sizeof(full_refs[0]) / sizeof(full_refs[0][0]));
				c++) {

			full_refs[r][c] = ref[r][c];

		}

	}

}

int DriveController::GetIndex() {

	return *ptr_index;

}
void DriveController::ResetIndex() {

	profile_index = 0;

}

