/*
 * lift.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

#include <iostream>
#include <string>
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include "Drive/DifferentialDrive.h"
#include "DriverStation.h"
#include <iostream>
#include <Encoder.h>
#include "lift.h"

LiftManager::LiftManager() {
	srx1 = new WPI_TalonSRX(10);

	this->stick = new Joystick { 0 };
}

void LiftManager::Lift() {
	if (stick->GetRawButton(5)) {
	srx1->Set(0.5);
	}
	else if (stick->GetRawButton(6)) {
		srx1->Set(-0.5);
	}
	else if (!stick->GetRawButton(5) and !stick->GetRawButton(6)) {
		srx1->Set(0);
	}
}


