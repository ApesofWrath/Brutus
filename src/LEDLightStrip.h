/*
 * LEDLightStrip.h
 *
 *  Created on: Jan 20, 2017
 *      Author: DriversStation
 */

#ifndef SRC_LEDLIGHTSTRIP_H_
#define SRC_LEDLIGHTSTRIP_H_

#include <LEDLightStrip.h>
#include <mindsensors.h>
#include <LEDLightStrip.h>
#include <iostream>
using mindsensors::CANLight;

class LEDLightStrip {
public:

	LEDLightStrip();
	void LEDGreen();
	void LEDBlue();
	void LEDYellow();
	void LEDRed();
	void LEDOff();
	void LEDStateMachine(bool gear_light_button, bool ball_light_button, bool gear_and_ball_light_button);

	CANLight *light;

};

#endif /* SRC_LEDLIGHTSTRIP_H_ */
