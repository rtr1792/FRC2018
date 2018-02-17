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
#include <Math.h>

LiftManager::LiftManager() {
	double kTimeoutMs = 10;
	double kPIDLoopIdx = 0;

	srx1 = new WPI_TalonSRX(10); //Has Encoder - Master
	srx2 = new WPI_TalonSRX(11); //Slave / Follower

	srx2->Set(ControlMode::Follower, 10);
	srx1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	srx1->SetSensorPhase(true);
	srx1->SetInverted(false);
	srx1->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);


	/* set the peak and nominal outputs, 12V means full */
	srx1->ConfigNominalOutputForward(0, kTimeoutMs);
	srx1->ConfigNominalOutputReverse(0, kTimeoutMs);
	srx1->ConfigPeakOutputForward(10, kTimeoutMs);
	srx1->ConfigPeakOutputReverse(-4 , kTimeoutMs);

	/* set closed loop gains in slot0 */
	srx1->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
	srx1->Config_kP(kPIDLoopIdx, 0.08, kTimeoutMs);
	srx1->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	srx1->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

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
	//Timer* timer;
	timer = new Timer();
	timer->Start();

	//Auto Lift
	DriveSRX1 =  new WPI_TalonSRX(1);
	DriveSRX2 =  new WPI_TalonSRX(4);

}
double xb;
double heatpreventmoveamount = 1000;
bool goback = false;
double oldpos = 0;
double dtvel = 0;
double possetto = 0;
int RisetoFall = -1;
int RisetoRise = -1;
double LiftVoltage = 0;
double LiftSpeed = 0;
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
		srx1->Set(ControlMode::PercentOutput, xb);
	}
	if(stick->GetRawButton(8)) {
		srx1->Set(ControlMode::Position, 36535); //Scale Height -2 Just Incase
		timer->Reset();
	}
	if (stick->GetRawButton(10)) {
		srx1->Set(ControlMode::Position, 10000); //Switch Height
		timer->Reset();
	}
	if (stick->GetRawButton(12)) {
		srx1->Set(ControlMode::Position, 0); //Bottom
		timer->Reset();
	}
	if(false){
		srx1->Set(ControlMode::Position, 0); //Neutral Height
	}

	*encoder = -srx2->GetSensorCollection().GetQuadraturePosition();
	frc::SmartDashboard::PutNumber("liftEnc",*encoder);
	frc::SmartDashboard::PutNumber("liftEncRot",*encoder/4096);

	if (xbox->GetRawButton(9)) {
		srx1->GetSensorCollection().SetQuadraturePosition(0,4);
	}

	frc::SmartDashboard::PutNumber("top limit",!limit2->Get());
	frc::SmartDashboard::PutNumber("bottom limit",!limit->Get());
		if(!limit2->Get()){
			srx1->GetSensorCollection().SetQuadraturePosition(36535, 10); //Set to Top - 10ms Allowed Time
		}

		if(!limit->Get()){
			srx1->GetSensorCollection().SetQuadraturePosition(0, 10); //Set to Zero - 10ms Allowed Time
		}

		/*if (!limit->Get() and -xbox->GetRawAxis(5) < 0) {
			srx1->Set(0);
			srx2->Set(0);
		}
		if (!limit->Get() and -xbox->GetRawAxis(5) > 0) {*/
			//srx1->Set(xb);		srx2->Set(ControlMode::PercentOutput, xb);

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

	//Heat Check Code
	if (timer->Get() > 25 && !goback) {
		timer->Reset();
		oldpos = -srx1->GetSensorCollection().GetQuadraturePosition();
		if(-srx1->GetSensorCollection().GetQuadraturePosition() < 1000){
			srx1->Set(ControlMode::Position, (-srx1->GetSensorCollection().GetQuadraturePosition()-heatpreventmoveamount)); // Go Down 1000
		}
		else{
			srx1->Set(ControlMode::Position, (-srx1->GetSensorCollection().GetQuadraturePosition()+heatpreventmoveamount)); // Go Up 1000
		}
	}
	if (goback && timer->Get() > 5){
		srx1->Set(ControlMode::Position, oldpos); // Go Back to Original
	}
	//End of Heat Check Code
	//AutoLift
	oldpos = -srx1->GetSensorCollection().GetQuadraturePosition();
	dtvel = fabs(DriveSRX1->GetSensorCollection().GetQuadratureVelocity()) + fabs(DriveSRX1->GetSensorCollection().GetQuadratureVelocity());
	possetto = srx1->GetClosedLoopTarget(0);
	if(possetto == 0 && dtvel > 0){
		srx1->Set(ControlMode::Position, 2000); //Neutral Height
	}
	if(xbox->GetRawButton(1) && possetto == 2000){
		srx1->Set(ControlMode::Position, 0);
	}
	//End of Auto Lift and Down
	//Check Encoder Health
	RisetoFall = srx1->GetSensorCollection().GetPulseWidthRiseToFallUs();
	RisetoRise = srx1->GetSensorCollection().GetPulseWidthRiseToRiseUs();
	frc::SmartDashboard::PutNumber("RisetoFall", RisetoFall);
	frc::SmartDashboard::PutNumber("RisetoRise", RisetoRise);
	if(RisetoFall == 0 || RisetoRise == 0){
		srx1->Set(ControlMode::PercentOutput, xb);
		//xbox->SetRumble(frc::GenericHID::kRightRumble, 100);
		//xbox->SetRumble(frc::GenericHID::kLeftRumble, 100);
	}
	//End of Encoder Health Check
	//775pro Health Keep
	LiftVoltage = srx1->GetMotorOutputVoltage();
	LiftSpeed = srx1->GetSensorCollection().GetQuadratureVelocity();
	if((LiftSpeed < 25 && -25 < LiftSpeed) && LiftVoltage > 4){
		srx1->Set(ControlMode::PercentOutput, 0.4); //Set to 2 Volts
	}
	//End of 775pro Health Keep

}

void LiftManager::Liftmove(int pos) {
	srx1->Set(ControlMode::Position, pos);
}


