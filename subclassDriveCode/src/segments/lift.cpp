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
//#include <PowerDistributionPanel.h>
#include "Timer.h"

LiftManager::LiftManager() {
	double kTimeoutMs = 10;
	double kPIDLoopIdx = 0;

	srx1 = new WPI_TalonSRX(10); //Slave
	srx2 = new WPI_TalonSRX(11); //Has Encoder - Master

	srx1->Set(ControlMode::Follower, 11);
	srx2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	srx2->SetSensorPhase(true);
	srx2->SetInverted(false);
	srx2->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);


	/* set the peak and nominal outputs, 12V means full */
	srx2->ConfigNominalOutputForward(0, kTimeoutMs);
	srx2->ConfigNominalOutputReverse(0, kTimeoutMs);
	srx2->ConfigPeakOutputForward(10, kTimeoutMs);
	srx2->ConfigPeakOutputReverse(-4 , kTimeoutMs);

	/* set closed loop gains in slot0 */
	srx2->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
	srx2->Config_kP(kPIDLoopIdx, 0.08, kTimeoutMs);
	srx2->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	srx2->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

	this->stick = new Joystick { 0 };
	xbox = new XboxController { 1 };

	limit = new DigitalInput { 1 };
	limit2 = new DigitalInput { 0 };

	liftValue = new int;
	button3 = new int;
	button4 = new int;
	encoder = new double;
	limits = new int;
	dlimit = new int;

	init = new int;
	limitOveride = new int;
	limitOveride2 = new int;

	//srx1->GetSensorCollection().SetQuadraturePosition(0,4);
	Timer* timer;
	timer = new Timer();

}

double xb;
//12,3
//limit depressed = 0
void LiftManager::Lift() {
	if (-xbox->GetRawAxis(5) < 0.05 and -xbox->GetRawAxis(5) > -0.05) {
		xb = 0;
	}
	else {
		xb = -xbox->GetRawAxis(5);
	}
	if(xbox->GetRawButton(5) and ((limit->Get() and limit2->Get()) or ((!limit->Get() and xb > 0) or (!limit2->Get() and xb < 0)))){
		srx2->Set(ControlMode::PercentOutput, xb);
	}
	if(stick->GetRawButton(8)){
		srx2->Set(ControlMode::Position, 36535); //Scale Height -2 Just Incase
	}
	if (stick->GetRawButton(10)) {
		srx2->Set(ControlMode::Position, 10000); //Switch Height
	}
	if (stick->GetRawButton(12)) {
		srx2->Set(ControlMode::Position, 0); //Bottom
	}
	if(false){
		srx2->Set(ControlMode::Position, 0); //Neutral Height
	}

	*encoder = -srx2->GetSensorCollection().GetQuadraturePosition();
	frc::SmartDashboard::PutNumber("liftEnc",*encoder);
	frc::SmartDashboard::PutNumber("liftEncRot",*encoder/4096);

	if (xbox->GetRawButton(9)) {
		srx2->GetSensorCollection().SetQuadraturePosition(0,4);
	}

	frc::SmartDashboard::PutNumber("top limit",!limit2->Get());
	frc::SmartDashboard::PutNumber("bottom limit",!limit->Get());
		if(!limit2->Get()){
			srx2->GetSensorCollection().SetQuadraturePosition(36535, 10); //Set to Top - 10ms Allowed Time
		}

		if(!limit->Get()){
			srx2->GetSensorCollection().SetQuadraturePosition(0, 10); //Set to Zero - 10ms Allowed Time
		}

		/*if (!limit->Get() and -xbox->GetRawAxis(5) < 0) {
			srx1->Set(0);
			srx2->Set(0);
		}
		if (!limit->Get() and -xbox->GetRawAxis(5) > 0) {*/
			//srx1->Set(xb);
			//srx2->Set(xb);
		//}
		/*if (!limit2->Get() and -xbox->GetRawAxis(5) > 0) {
			srx1->Set(0);
			srx2->Set(0);
		}
		if (!limit2->Get() and -xbox->GetRawAxis(5) < 0) {
			srx1->Set(xb);
			srx2->Set(xb);
		}
		if (limit->Get() and limit->Get()) {
			srx1->Set(xb);
			srx2->Set(xb);
		}*/

		if (!limit->Get() and -xbox->GetRawAxis(5) < 0) {
			srx2->Set(0);
		}
		if (!limit2->Get() and -xbox->GetRawAxis(5) > 0) {
			srx2->Set(0);
		}


	double p1 = srx1->Get();
	double p2 = srx2->Get();

	frc::SmartDashboard::PutNumber("lm1",p1);
	frc::SmartDashboard::PutNumber("lm2",p2);

	double c1 = srx1->GetOutputCurrent();
	double c2 = srx2->GetOutputCurrent();

	frc::SmartDashboard::PutNumber("lc1",c1);
	frc::SmartDashboard::PutNumber("lc2",c2);

	//10003 switch
	//35780
}


