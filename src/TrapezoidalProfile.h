/*
 * TrapezoidalProfile.h
 *
 *  Created on: Sep 2, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>
#include <list>
#include <vector>

#ifndef SRC_TRAPEZOIDALPROFILE_H_
#define SRC_TRAPEZOIDALPROFILE_H_

class TrapezoidalProfile {
public:

	TrapezoidalProfile(double max_vel, double max_acc, double time_step);
	std::vector<std::vector<double>> CreateProfile(double init_pos, double ref);

};

#endif
