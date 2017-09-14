/*
 * Vision.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: DriversStation
 */

#include <Vision.h>

Vision::Vision() {


}

double Vision::findAzimuth() {

	NetworkTable::SetClientMode();
	NetworkTable::SetTeam(668);

	auto table = NetworkTable::GetTable("SmartDashboard");

	double azimuth = table->GetNumber("Azimuth", 0.0);

	return azimuth;

}

double Vision::findDistance() {

	NetworkTable::SetClientMode();
	NetworkTable::SetTeam(668);

	auto table = NetworkTable::GetTable("SmartDashboard");

	double distance = table->GetNumber("Distance", 0.0);

	return distance;

}
