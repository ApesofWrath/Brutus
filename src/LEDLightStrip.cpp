/*
 * LEDLightStrip.cpp
 *
 *  Created on: Jan 20, 2017
 *      Author: DriversStation
 */

#include <LEDLightStrip.h>
#include <mindsensors.h>

const int gear_state = 0;
const int ball_state = 1;
const int gear_and_ball_state = 2;
const int off_state = 3;


int led_state = off_state;

using mindsensors::CANLight;


LEDLightStrip::LEDLightStrip() {

	//light = new CANLight(5);

}
void LEDLightStrip::LEDGreen(){

//	light->ShowRGB(0, 255, 0);

}
void LEDLightStrip::LEDBlue(){

	//light->ShowRGB(0, 0, 255);

}

void LEDLightStrip::LEDRed(){

	//light->ShowRGB(255, 0, 0);

}

void LEDLightStrip::LEDOff(){

	//light->ShowRGB(0, 0, 0);

}

void LEDLightStrip::LEDStateMachine(bool gear_light_button, bool ball_light_button, bool gear_and_ball_light_button){


	switch(led_state){

	case gear_state:

		LEDGreen();
		std::cout << "green" << std::endl;

		if(!gear_light_button){

			led_state = off_state;

		}

		break;

	case ball_state:

		LEDBlue();
		std::cout << "blue" << std::endl;

		if(!ball_light_button){

			led_state = off_state;

		}

		break;

	case gear_and_ball_state:

		LEDRed();
		std::cout << "red" << std::endl;

		if(!gear_and_ball_light_button){

			led_state = off_state;

		}

		break;

	case off_state:

		LEDOff();
		std::cout << "off" << std::endl;
		if (gear_light_button){

			led_state = gear_state;

		}else if (ball_light_button){

			led_state = ball_state;

		}else if (gear_and_ball_light_button){

			led_state = gear_and_ball_state;

		}

		break;

	}

}




