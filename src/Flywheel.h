/*
 * Flywheel.h
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>
#include <CANTalon.h>
#include <Timer.h>

#ifndef SRC_FLYWHEEL_H_
#define SRC_FLYWHEEL_H_

class Flywheel {
public:

	const int stop_state_h = 0;
	const int spin_state_h = 1;
	int flywheel_state = stop_state_h;


	CANTalon *canTalonFlywheelRight, *canTalonFlywheelLeft;

	Flywheel();

	void FlywheelStateMachine();
	void Spin(int ref);
	void Stop();
	bool IsAtSpeedLeft();
	bool IsAtSpeedRight();
	void StartThread();
	void DisableThreads();
	static void SpinWrapper(Flywheel *fw, int ref, bool *active);
	double GetSpeedRight();
	double GetSpeedLeft();
	double FlywheelValueLeft();
	double FlywheelValueRight();
	void SetGoal(double joyVal);
	int GetGoal();
};

#endif
