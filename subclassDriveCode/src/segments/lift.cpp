/*
 * lift.cpp
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
#include "lift.h"
#include <DigitalInput.h>
#include <PowerDistributionPanel.h>

LiftManager::LiftManager() {
	srx1 = new WPI_TalonSRX(10);
	srx2 = new WPI_TalonSRX(11);

	this->stick = new Joystick { 0 };
	xbox = new XboxController { 1 };

	limit = new DigitalInput { 9 };
	limit2 = new DigitalInput { 8 };

	liftValue = new int;
	button3 = new int;
	button4 = new int;
	encoder = new double;
	limits = new int;
	dlimit = new int;

	zero = new int;
	one = new int;
	two = new int;
	three = new int;
	four = new int;
	mone = new int;
	init = new int;
	limitOveride = new int;
	limitOveride2 = new int;

	pdp = new PowerDistributionPanel;

	srx1->GetSensorCollection().SetQuadraturePosition(0,4);
}
//12,3
//limit depressed = 0
void LiftManager::Lift() {
	*zero = 0;
	*one = 1;
	*two = 2;
	*three = 3;
	*four = 4;
	*mone = -1;

	if (*init > *one or *init < *zero) {
		*init = 0;
	}

	if (*init == *zero) {
		*liftValue = -1;
		*init = 1;
	}

	if (stick->GetRawButton(11)) {
		*liftValue = 0;
	}
	if (stick->GetRawButton(12)) {
		*liftValue = 1;
	}
	if (stick->GetRawButton(10)) {
		*liftValue = 2;
	}
	if (stick->GetRawButton(8)) {
		*liftValue = 3;
	}
	if (stick->GetRawButton(7)) {
		*liftValue = 4;
	}
	if (stick->GetRawButton(9)) {
		*liftValue = -1;
	}


	if (stick->GetRawButton(4) and (*button3 == *one)) {
		*liftValue = *liftValue - 1;
		*button3 = 0;
	}
	else if (!stick->GetRawButton(4)) {
		*button3 = 1;
	}

	if (stick->GetRawButton(3) and (*button4 == *one)) {
		*liftValue = *liftValue + 1;
		*button4 = 0;
	}
	else if (!stick->GetRawButton(3)) {
		*button4 = 1;
	}

	if ((*liftValue == *zero) and limit->Get() and -srx2->GetSensorCollection().GetQuadraturePosition() > 500) {
		srx1->Set(-0.5);
		srx2->Set(-0.5);
	}
	else if ((*liftValue == *zero) and (!limit->Get() or -srx2->GetSensorCollection().GetQuadraturePosition() < 500)) {
		srx1->Set(0);
		srx2->Set(0);
	}

	if ((*liftValue == *one) and -srx2->GetSensorCollection().GetQuadraturePosition() < 4000 and limit2->Get()) {
		srx1->Set(0.5);
		srx2->Set(0.5);
	}
	else if ((*liftValue == *one) and -srx2->GetSensorCollection().GetQuadraturePosition() > 5000 and limit->Get()) {
		srx1->Set(-0.2);
		srx2->Set(-0.2);
	}
	else if (((*liftValue == *one) and (-srx2->GetSensorCollection().GetQuadraturePosition() < 5000 and -srx2->GetSensorCollection().GetQuadraturePosition() > 4000))) {
		srx1->Set(0);
		srx2->Set(0);
	}

	if ((*liftValue == *two) and -srx2->GetSensorCollection().GetQuadraturePosition() < 10003 and limit2->Get()) {
		srx1->Set(0.5);
		srx2->Set(0.5);
	}
	else if ((*liftValue == *two) and -srx2->GetSensorCollection().GetQuadraturePosition() > 11003 and limit->Get()) {
		srx1->Set(-0.2);
		srx2->Set(-0.2);
	}
	else if ((*liftValue == *two) and -srx2->GetSensorCollection().GetQuadraturePosition() < 11003 and -srx2->GetSensorCollection().GetQuadraturePosition() > 10003) {
		srx1->Set(0);
		srx2->Set(0);
	}

	if ((*liftValue == *three) and -srx2->GetSensorCollection().GetQuadraturePosition() < 20003 and limit2->Get()) {
		srx1->Set(0.5);
		srx2->Set(0.5);
	}
	else if ((*liftValue == *three) and -srx2->GetSensorCollection().GetQuadraturePosition() > 25003 and limit->Get()) {
		srx1->Set(-0.2);
		srx2->Set(-0.2);
	}
	else if ((*liftValue == *three) and -srx2->GetSensorCollection().GetQuadraturePosition() > 20003 and -srx2->GetSensorCollection().GetQuadraturePosition() < 25003) {
		srx1->Set(0);
		srx2->Set(0);
	}

	if ((*liftValue == *four) and -srx2->GetSensorCollection().GetQuadraturePosition() < 35680 and limit2->Get()) {
		srx1->Set(0.35);
		srx2->Set(0.35);
	}
	else if ((*liftValue == *four) and -srx2->GetSensorCollection().GetQuadraturePosition() > 35800 and limit2->Get()) {
		srx1->Set(-0.2);
		srx2->Set(-0.2);
	}
	else if (((*liftValue == *four) and (-srx2->GetSensorCollection().GetQuadraturePosition() < 5000 and -srx2->GetSensorCollection().GetQuadraturePosition() > 35780))) {
		srx1->Set(0.01);
		srx2->Set(0.01);
	}

	frc::SmartDashboard::PutNumber("lift",*liftValue);

	*encoder = -srx2->GetSensorCollection().GetQuadraturePosition();
	frc::SmartDashboard::PutNumber("liftEnc",*encoder);

	if (xbox->GetRawButton(9)) {
		srx2->GetSensorCollection().SetQuadraturePosition(0,4);
	}

	frc::SmartDashboard::PutNumber("top limit",!limit2->Get());
	frc::SmartDashboard::PutNumber("bottom limit",!limit->Get());


	if (!limit->Get() and (*limits == *one)) {
		srx2->GetSensorCollection().SetQuadraturePosition(0,4);
		*liftValue = 0;
		*limits = 0;
	}
	else if (limit->Get()) {
		*limits = 1;
	}

	if (!limit2->Get() and (*dlimit == *one)) {
		*dlimit = 0;
	}
	else if (limit2->Get()) {
		*dlimit = 1;
	}

	if (-xbox->GetRawAxis(5) < 1 and -xbox->GetRawAxis(5) > -0.2 and ((*liftValue > *four) or (*liftValue == *mone)) and (((*limitOveride == *one) and (*limitOveride2 == *one)) or (limit->Get() and limit2->Get())) ) {
		srx1->Set(-xbox->GetRawAxis(5));
		srx2->Set(-xbox->GetRawAxis(5));
	}
	else if (-xbox->GetRawAxis(5) > 1 and (*liftValue > *four) and (((*limitOveride == *one) and (*limitOveride2 == *one)) or (limit->Get() and limit2->Get()))) {
		srx1->Set(1);
		srx2->Set(1);
	}
	else if (-xbox->GetRawAxis(5) < -0.2 and (*liftValue > *four) and (((*limitOveride == *one) and (*limitOveride2 == *one)) or (limit->Get() and limit2->Get()))) {
		srx1->Set(-0.2);
		srx2->Set(-0.2);
	}
	else if (!limit->Get() and ((*liftValue > *four) or (*liftValue == *mone)) and -xbox->GetRawAxis(5) < 0){
		srx1->Set(0);
		srx2->Set(0);
		*limitOveride = 0;
	}
	else if (!limit2->Get() and ((*liftValue > *four) or (*liftValue == *mone)) and -xbox->GetRawAxis(5) > 0){
		srx1->Set(0);
		srx2->Set(0);
		*limitOveride2 = 0;
	}
	else if (!limit->Get() and  -xbox->GetRawAxis(5) > 0) {
		*limitOveride = 1;
	}
	else if (!limit2->Get() and  -xbox->GetRawAxis(5) < 0) {
		*limitOveride2 = 1;
	}


	double p1 = srx1->Get();
	double p2 = srx2->Get();

	frc::SmartDashboard::PutNumber("lm1",p1);
	frc::SmartDashboard::PutNumber("lm2",p2);

	double c1 = srx1->GetOutputCurrent();
	double c2 = srx2->GetOutputCurrent();

	frc::SmartDashboard::PutNumber("lc1",c1);
	frc::SmartDashboard::PutNumber("lc2",c2);

	double pc1 = pdp->GetCurrent(3);
	double pc2 = pdp->GetCurrent(12);

	frc::SmartDashboard::PutNumber("pc1",pc1);
	frc::SmartDashboard::PutNumber("pc2",pc2);
	//10003 switch
	//35780

	if (xbox->GetRawButton(10)) {
		*liftValue = -1;
	}
}


