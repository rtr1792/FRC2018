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
#include <DigitalInput.h>

LiftManager::LiftManager() {
	srx1 = new WPI_TalonSRX(10);

	this->stick = new Joystick { 0 };
	xbox = new XboxController { 1 };

	limit = new DigitalInput { 9 };
	limit2 = new DigitalInput { 8 };

	liftValue = new int;
	button3 = new bool;
	button4 = new bool;
	encoder = new double;

	zero = new int;
	one = new int;
	two = new int;
	three = new int;
	four = new int;
}
//limit depressed = 0
void LiftManager::Lift() {
	*zero = 0;
	*one = 1;
	*two = 2;
	*three = 3;
	*four = 4;

	if (xbox->GetRawButton(10)) {
		*liftValue = 0;
	}

	if (xbox->GetRawButton(3) and *button3) {
		*liftValue = *liftValue - 1;
		*button3 = false;
	}
	else if (!xbox->GetRawButton(3)) {
		*button3 = true;
	}

	if (xbox->GetRawButton(4) and button4) {
		*liftValue = *liftValue + 1;
		*button4 = false;
	}
	else if (!xbox->GetRawButton(4)) {
		*button4 = true;
	}

	if ((*liftValue == *zero) and limit->Get() and srx1->GetSensorCollection().GetQuadraturePosition() > 1000) {
		srx1->Set(-0.2);
	}
	else if ((*liftValue == *zero) and (!limit->Get() or srx1->GetSensorCollection().GetQuadraturePosition() < 1000)) {
		srx1->Set(0);
	}

	if ((*liftValue == *one) and srx1->GetSensorCollection().GetQuadraturePosition() < 1001) {
		srx1->Set(0.2);
	}
	else if ((*liftValue == *one) and srx1->GetSensorCollection().GetQuadraturePosition() > 2000) {
		srx1->Set(-0.2);
	}
	else if ((*liftValue == *one) and srx1->GetSensorCollection().GetQuadraturePosition() < 2001 and srx1->GetSensorCollection().GetQuadraturePosition() > 1001) {
		srx1->Set(0);
	}

	if ((*liftValue == *two) and srx1->GetSensorCollection().GetQuadraturePosition() < 2001) {
		srx1->Set(0.2);
	}
	else if ((*liftValue == *two) and srx1->GetSensorCollection().GetQuadraturePosition() > 3000) {
		srx1->Set(-0.2);
	}
	else if ((*liftValue == *two) and srx1->GetSensorCollection().GetQuadraturePosition() < 3001 and srx1->GetSensorCollection().GetQuadraturePosition() > 2001) {
		srx1->Set(0);
	}

	if ((*liftValue == *three) and srx1->GetSensorCollection().GetQuadraturePosition() < 3001) {
		srx1->Set(0.2);
	}
	else if ((*liftValue == *three) and srx1->GetSensorCollection().GetQuadraturePosition() > 4000) {
		srx1->Set(-0.2);
	}
	else if ((*liftValue == *three) and srx1->GetSensorCollection().GetQuadraturePosition() > 3001 and srx1->GetSensorCollection().GetQuadraturePosition() < 4001) {
		srx1->Set(0);
	}

	if ((*liftValue == *four) and srx1->GetSensorCollection().GetQuadraturePosition() < 4001 and limit2->Get()) {
		srx1->Set(0.2);
	}
	else if ((*liftValue == *four) and srx1->GetSensorCollection().GetQuadraturePosition() > 5000 and limit2->Get()) {
		srx1->Set(-0.2);
	}
	else if (!limit2->Get() or ((*liftValue == *four) and (srx1->GetSensorCollection().GetQuadraturePosition() < 5000 and srx1->GetSensorCollection().GetQuadraturePosition() > 4001))) {
		srx1->Set(0);
	}

	frc::SmartDashboard::PutNumber("lift",*liftValue);

	*encoder = srx1->GetSensorCollection().GetQuadraturePosition();
	frc::SmartDashboard::PutNumber("liftEnc",*encoder);

	if (xbox->GetRawButton(9)) {
		srx1->GetSensorCollection().SetQuadraturePosition(0,4);
	}

	frc::SmartDashboard::PutNumber("top limit",!limit2->Get());
	frc::SmartDashboard::PutNumber("bottom limit",!limit->Get());


	if (!limit->Get()) {
		srx1->GetSensorCollection().SetQuadraturePosition(0,4);
		*liftValue = 0;
	}

	/*
	if (xbox->GetRawButton(3) and limit->Get()) {
	srx1->Set(0.5);
	}
	else if (xbox->GetRawButton(4) and limit2->Get()) {
		srx1->Set(-0.5);
	}
	else if (!xbox->GetRawButton(3) and !xbox->GetRawButton(4)) {
		srx1->Set(0);
	}
	else if (!limit->Get() and !xbox->GetRawButton(4)) {
		srx1->Set(0);
	}
	else if (!limit2->Get() and !xbox->GetRawButton(3)) {
		srx1->Set(0);
	}
	frc::SmartDashboard::PutNumber("limit",limit->Get());

	double position = srx1->GetSensorCollection().GetQuadraturePosition();
	frc::SmartDashboard::PutNumber("lift position", position);

	if (!limit->Get()) {
		srx1->GetSensorCollection().SetQuadraturePosition(0,4);
	}
	*/
}


