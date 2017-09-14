/*
 * Vision.h
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */
#include <WPILib.h>
#ifndef SRC_VISION_H_
#define SRC_VISION_H_

class Vision {
public:

	Vision();
	double findAzimuth();
	double findDistance();

};

#endif
