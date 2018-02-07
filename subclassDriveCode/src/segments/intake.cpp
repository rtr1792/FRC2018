/*
 * intake.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <iostream>
#include <Encoder.h>
//#include <segments/intake.h>
#include "intake.h"
#include <Ultrasonic.h>

IntakeManager::IntakeManager() {
	srx1 = new WPI_TalonSRX(7);
	srx2 = new WPI_TalonSRX(8);

	this->stick = new Joystick { 0 };
	this->xbox = new XboxController { 1 };

	ult = new Ultrasonic(7,6);
	ult->SetAutomaticMode(true);
	ult->SetEnabled(true);

	ult2 = new Ultrasonic(5,4);
	ult2->SetAutomaticMode(true);
	ult2->SetEnabled(true);

	rd = new int;
	ld = new int;
	zero = new int;
	one = new int;
	reverse = new int;
}

void IntakeManager::Intake() {
	*zero = 0;
	*one = 1;

	frc::SmartDashboard::PutNumber("test",1);

	if (xbox->GetRawButton(2) and !xbox->GetRawButton(3)) { //and (*ld == *zero) and (*rd == *zero)) {
		srx1->Set(1);
		srx2->Set(-1);
	}
	else if (!xbox->GetRawButton(2) and xbox->GetRawButton(3)) {
		srx1->Set(0.5);
		srx2->Set(-0.5);
	}
	else if (xbox->GetRawButton(1)) {
		srx2->Set(0.5);
		srx1->Set(-0.5);
	}
	else if (!xbox->GetRawButton(1) and !xbox->GetRawButton(2) and !xbox->GetRawButton(3)) {
		srx1->Set(0);
		srx2->Set(0);
	}
/*
	if ((*ld == *one) and (*reverse == *zero) and xbox->GetRawButton(1)) {
		srx1->Set(0.5);
		srx2->Set(0.3);
	}
	else if ((*rd == *one) and (*reverse == *zero) and xbox->GetRawButton(1)) {
		srx1->Set(0.3);
		srx2->Set(0.5);
	} */

	double mm = ult->GetRangeMM();
	frc::SmartDashboard::PutNumber("mm",mm);
	double ultd = ult->GetRangeInches();
	frc::SmartDashboard::PutNumber("In",ultd);

	double mm2 = ult2->GetRangeMM();
	frc::SmartDashboard::PutNumber("mm2",mm2);
	double ultd2 = ult2->GetRangeInches();
	frc::SmartDashboard::PutNumber("In2",ultd2);

/*	if (ultd > ultd2 + 2) {
		*ld = 1;
	}
	else {
		*ld = 0;
	}

	if (ultd2 > ultd + 2) {
		*rd = 1;
	}
	else {
		*rd = 0;
	}
*/
}


