/*
 * intake.cpp
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
//#include <segments/intake.h>
#include "intake.h"

IntakeManager::IntakeManager() {
	srx1 = new WPI_TalonSRX(7);
	srx2 = new WPI_TalonSRX(8);

	this->stick = new Joystick { 0 };
	this->xbox = new XboxController { 1 };

}

void IntakeManager::Intake() {
	if (xbox->GetRawButton(1)) {
		srx1->Set(0.5);
		srx2->Set(0.5);
	}
	else if (xbox->GetRawButton(2)) {
		srx2->Set(-0.5);
		srx1->Set(-0.5);
	}
	else if (!xbox->GetRawButton(1) and !xbox->GetRawButton(2)) {
		srx1->Set(0);
		srx2->Set(0);
	}
}


