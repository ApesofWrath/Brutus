/*
 * DriveController.h
 *
 *  Created on: Jan 9, 2017
 *      Author: Apes of Wrath
 */
#include <WPILib.h>
#include <CANTalon.h>
#include <RobotDrive.h>
#include <Joystick.h>
#include <AHRS.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <Timer.h>
#include <Vision.h>
#include <math.h>
#include <TrapezoidalProfile.h>

#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_


class DriveController {
public:

	RobotDrive *robotDrive;
	CANTalon *canTalonFrontLeft, *canTalonFrontRight, *canTalonBackRight,
			 *canTalonBackLeft, *canTalonKicker;
	AHRS *ahrs;
	DoubleSolenoid *kickerPiston;

	DriveController(Vision *vis);

	void StartTeleopThreads(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc);
	void StartAutonThreads();
	void EndThreads();

	void Drive(double ref_kick, double ref_right, double ref_left, double ref_yaw, double k_p_right, double k_p_left, double k_p_kick,
			double k_p_yaw, double k_d_yaw, double k_d_right, double k_d_left, double k_d_kick, double target_vel_left, double target_vel_right, double target_vel_kick);
	void DrivePID();
	void HDrive(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_fc);
	void HeadingPID(Joystick *joyWheel);
	void VisionP(double ref);
	void AutoVisionTrack();

	void StopAll();
	void KickerUp();
	void KickerDown();
	void ZeroEncs();
	void ZeroI();
	int GetIndex();
	void ResetIndex();
	void SetRef(double refs[][15]);
	double GetRef();
	void SetInitHeading();
	void SetAngle();
	void SetDist();
	bool CheckIfNull();
	void StoreEncValues();
	void SetEncValues();
	void ResetVisionState();

	static void DrivePIDWrapper(DriveController *driveController);
	static void HDriveWrapper(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc, DriveController *driveController);

	void DisableTeleopThreads();
	void DisableAutonThreads();
};

#endif /* SRC_DRIVECONTROLLER_H_ */
