/*
 * GearPickup.h
 *
 *  Created on: Jul 15, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>
#include <CANTalon.h>
#include <TrapezoidalProfile.h>

#ifndef SRC_GEARPICKUP_H_
#define SRC_GEARPICKUP_H_

class GroundPickup {
public:

	GroundPickup();

	void GroundPickupStateMachine();

	void SpinIn();
	void SpinOut();
	void StopSpin();


	void ZeroPos();
	double GetPos();
	void SetPos(double pos);

	void MoveArm(double ref, double profileVelocity);

	void ClearIAccum();
	bool IsAtPosition();
	double GetVel();
	int GetEncPos();
	double CheckSpinCurrent();

	static void MoveWrapper(GroundPickup *gp, int *ref_pos);

	void StartThreads();
	void DisableThreads();

	void SetIndex(int index);
	int GetIndex();

	CANTalon *canTalonFloorPickupArm;
	CANTalon *canTalonPickupWheel;

	const double MAX_SPIN_CURRENT = 6.0;

	int gearpickup_index = 0;

	const int arm_up_state_h = 0;
	const int spin_out_state_h = 1;
	const int spin_in_state_h = 2;
	const int spin_stop_state_h = 3;
	const int arm_down_state_h = 4;
	const int arm_up_spin_in_state_h = 5;
	const int arm_up_spin_out_state_h = 6;
	const int arm_down_spin_in_state_h = 7;
	const int arm_down_spin_out_state_h = 8;
	const int arm_start_state_h = 9;
	const int arm_start_spin_in_state_h = 10;

	int ground_pickup_state = arm_up_state_h; //first state in teleop



};

#endif
