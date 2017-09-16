/*
 * TrapezoidalProfile.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: DriversStation
 */

#include <TrapezoidalProfile.h>

double ramp_time = 0.0;
double ramp_dis = 0.0;

double max_acceleration = 0.0;
double max_velocity = 0.0;

double iterations = 0.0;
double time_dt = 0.00001;
double interval = 0.0;

std::vector<double> positions = { 0 }; //first points will be 0
std::vector<double> velocity = { 0 };

TrapezoidalProfile::TrapezoidalProfile(double max_vel, double max_acc,
		double time_step) {

	max_velocity = max_vel;
	max_acceleration = max_acc;

	interval = time_dt / time_step;
}

std::vector<std::vector<double>> TrapezoidalProfile::CreateProfile(
		double init_pos, double ref) {

	double acc = 0.0;
	double vel = 0.0;
	double pos = init_pos;

	double last_vel = 0.0;
	double last_pos = init_pos;

	double time = 0.0;

	int counter = 0;

	std::vector<std::vector<double> > matrix; //new matrix every time because .push_back adds rows

	if (ref >= init_pos) {
		while (pos < ref) {

			ramp_time = vel / max_acceleration;
			ramp_dis = 0.5 * (vel * ramp_time);

			if ((ref - ramp_dis) <= pos) { //should
				acc = -1.0 * max_acceleration;
			} else if (vel < max_velocity) {
				acc = max_acceleration;
			} else {
				acc = 0.0;
			}

			pos = last_pos + (vel * time_dt);
			last_pos = pos;

			vel = last_vel + (acc * time_dt);
			last_vel = vel;

			counter++;
			time += time_dt;

			if (counter == 1000) { //TODO: check if need to cast to int
				positions.push_back(pos);
				velocity.push_back(vel);
				counter = 0;
			}
		}
	}
	else if (ref < init_pos) {
		while (pos > ref) {

			ramp_time = vel / max_acceleration;
			ramp_dis = 0.5 * (vel * ramp_time);

			if ((ramp_dis - ref) >= pos) { //should
				acc = 1.0 * max_acceleration;
			} else if (vel > (-1.0 * max_velocity)) {
				acc = -1.0 * max_acceleration;
			} else {
				acc = 0.0;
			}

			pos = last_pos + (vel * time_dt);
			last_pos = pos;

			vel = last_vel + (acc * time_dt);
			last_vel = vel;

			counter++;
			time += time_dt;

			if (counter == 1000) { //TODO: check if need to cast to int
				positions.push_back(pos);
				velocity.push_back(vel);
				counter = 0;
			}
		}
	}

	matrix.push_back(positions); //first vector,  row 0
	matrix.push_back(velocity); //second vector, row 1

	return matrix;

}

