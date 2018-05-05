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
#include <Robot.h>

LiftManager::LiftManager() {
	double kTimeoutMs = 10;
	double kPIDLoopIdx = 0;

	srxLiM = new WPI_TalonSRX(10); //Has Encoder - Master
	srxLiS = new WPI_TalonSRX(11); //Slave / Follower

	srxLiS->Set(ControlMode::Follower, 10);
	srxLiM->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	srxLiM->SetSensorPhase(true);
	srxLiM->SetInverted(false);
	srxLiM->ConfigAllowableClosedloopError(kPIDLoopIdx, 0, kTimeoutMs);


	/* set the peak and nominal outputs, 12V means full */
	srxLiM->ConfigNominalOutputForward(0, kTimeoutMs);
	srxLiM->ConfigNominalOutputReverse(0, kTimeoutMs);
	srxLiM->ConfigPeakOutputForward(1, kTimeoutMs);
	srxLiM->ConfigPeakOutputReverse(-0.5 , kTimeoutMs);

	/* set closed loop gains in slot0 */
	srxLiM->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
	srxLiM->Config_kP(kPIDLoopIdx, 16, kTimeoutMs);
	srxLiM->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	srxLiM->Config_kD(kPIDLoopIdx, 200, kTimeoutMs);


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

	//srxLiM->GetSensorCollection().SetQuadraturePosition(0,4);
	//Timer* timer;
	timer = new Timer();
	timer->Start();

	//Auto Lift
	DriveSRX1 =  new WPI_TalonSRX(1);
	DriveSRX2 =  new WPI_TalonSRX(4);

}
double xb;
double heatpreventmoveamount = 4000;
bool goback = false;
double oldpos = 0;
double pos = 0;
double dtvel = 0;
double possetto = 0;
int RisetoFall = -1;
int RisetoRise = -1;
double LiftVoltage = 0;
double LiftSpeed = 0;
double toplimit = 0;
double bottomlimit = 0;
int pov;
int timesec = 0;
bool firstrun = true;
int liftlocation = 180;
bool limitWorking;
//12,3
//limit depressed = 0
void LiftManager::Lift(int scaleheight, int switchheight, int driveheight) {
	limitWorking = limit->StatusIsFatal();
	frc::SmartDashboard::PutBoolean("limitWorking",limitWorking);

	pov = xbox->GetPOV();
	timesec = timer->Get();
	frc::SmartDashboard::PutNumber("Timer", timesec);

	//deadband
	if (-xbox->GetRawAxis(5) < 0.05 and -xbox->GetRawAxis(5) > -0.05) {
		xb = 0;
	}
	else {
		xb = -xbox->GetRawAxis(5);
	}

//human control
	if(xbox->GetRawButton(5) and ((limit->Get() and limit2->Get()) or ((!limit->Get() and xb > 0) or (!limit2->Get() and xb < 0)))){
		srxLiM->Set(ControlMode::PercentOutput, xb);
	}
	//pid control
	if(pov == 0) {
		srxLiM->Set(ControlMode::Position, scaleheight); //Scale Height -2 Just Incase
		timer->Reset();
		goback = false;
		liftlocation = 0;
	}
	if (pov == 90) {
		srxLiM->Set(ControlMode::Position, switchheight); //Switch Height
		timer->Reset();
		goback = false;
		liftlocation = 90;
	}
	if(pov == 270){
		srxLiM->Set(ControlMode::Position, driveheight); //Just Above Ground
		timer->Reset();
		goback = false;
		liftlocation = 270;
	}
	if (pov == 180) {
		srxLiM->Set(ControlMode::Position, 0); //Bottom
		timer->Reset();
		goback = false;
		liftlocation = 180;
	}
	*encoder = -srxLiM->GetSensorCollection().GetQuadraturePosition();
	frc::SmartDashboard::PutNumber("liftEnc",*encoder);
//reset encoder
	if (xbox->GetRawButton(9)) {
		srxLiM->GetSensorCollection().SetQuadraturePosition(0,4);
	}

	//Auto Lift
	if(!xbox->GetRawButton(5) && !xbox->GetRawButton(6)){
		if(liftlocation == 180){
			if(xbox->GetRawButton(1) || xbox->GetRawButton(2) || xbox->GetRawButton(3)){
				srxLiM->Set(ControlMode::Position, 0);
			}
			else{
				srxLiM->Set(ControlMode::Position, 4096);
			}
		}
	}
	if(xbox->GetRawButton(6) && !xbox->GetRawButton(5)){
		if(liftlocation == 180){
			srxLiM->Set(ControlMode::Position, 0);
		}
	}

	frc::SmartDashboard::PutNumber("top limit",!limit2->Get());
	frc::SmartDashboard::PutNumber("bottom limit",!limit->Get());
	//limit stops
		if(!limit2->Get()){
			//srxLiM->Set(ControlMode::Position, 34000);
			srxLiM->GetSensorCollection().SetQuadraturePosition(-scaleheight, 10); //Set to Zero - 10ms Allowed Time

		}

		if(!limit->Get()){
			//srxLiM->Set(ControlMode::Position, 0); //Bottom
			//srxLiM->GetSensorCollection().SetQuadraturePosition(-20, 10); //Set to Zero - 10ms Allowed Time
		}
		if(!limit2->Get() && !limit->Get()){ //Incase both Limits die switch to manual
			srxLiM->Set(ControlMode::PercentOutput, xb);
			xbox->SetRumble(frc::GenericHID::kLeftRumble, 10);
		}


	double p1 = srxLiM->Get();
	double p2 = srxLiS->Get();

	frc::SmartDashboard::PutNumber("lm1",p1);
	frc::SmartDashboard::PutNumber("lm2",p2);

	double c1 = srxLiM->GetOutputCurrent();
	double c2 = srxLiS->GetOutputCurrent();

	frc::SmartDashboard::PutNumber("lc1",c1);
	frc::SmartDashboard::PutNumber("lc2",c2);

	//10003 switch
	//35780

	//Heat Check Code
	/*
	if(timesec > 25 && firstrun){
		firstrun = false;
	}
	if (timesec > 15 && !goback && !firstrun && !xbox->GetRawButton(5)) {
		timer->Reset();
		pos = -srxLiM->GetSensorCollection().GetQuadraturePosition();
		if(-srxLiM->GetSensorCollection().GetQuadraturePosition() < 1000){
			oldpos = -srxLiM->GetSensorCollection().GetQuadraturePosition();
			srxLiM->Set(ControlMode::Position, (pos-heatpreventmoveamount)); // Go Down 1000
			goback = true;
		}
		else{
			oldpos = -srxLiM->GetSensorCollection().GetQuadraturePosition();
			srxLiM->Set(ControlMode::Position, (pos+heatpreventmoveamount)); // Go Up 1000
			goback = true;
		}
	}
	if (goback && timer->Get() > 5 && !firstrun && !xbox->GetRawButton(5)){
		srxLiM->Set(ControlMode::Position, pos); // Go Back to Original
		goback = false;
		timer->Reset();
	}
	frc::SmartDashboard::PutBoolean("Goback", goback);
	//End of Heat Check Code
	 */

	//Check Encoder Health
	RisetoFall = srxLiM->GetSensorCollection().GetPulseWidthRiseToFallUs();
	RisetoRise = srxLiM->GetSensorCollection().GetPulseWidthRiseToRiseUs();
	frc::SmartDashboard::PutNumber("RisetoFall", RisetoFall);
	frc::SmartDashboard::PutNumber("RisetoRise", RisetoRise);
	if(RisetoFall == 0 || RisetoRise == 0){
		srxLiM->Set(ControlMode::PercentOutput, xb);
		xbox->SetRumble(frc::GenericHID::kRightRumble, 100);
		//xbox->SetRumble(frc::GenericHID::kLeftRumble, 100);
	}
	else{

	}
	//End of Encoder Health Check
	//775pro Health Keep
	/*
	LiftVoltage = srxLiM->GetMotorOutputVoltage();
	frc::SmartDashboard::PutNumber("LiftMotorVoltage1", LiftVoltage);
	LiftSpeed = srxLiM->GetSensorCollection().GetQuadratureVelocity();
	if((LiftSpeed < 25 && -25 < LiftSpeed) && LiftVoltage > 4){
		//srxLiM->Set(ControlMode::PercentOutput, 0.4); //Set to 2 Volts
	}
	//End of 775pro Health Keep
	*/
	double PIDError = srxLiM->GetClosedLoopError(0);
	frc::SmartDashboard::PutNumber("PIDError", PIDError);

}
//allows the auto to control the lift
void LiftManager::Liftmove(int pos, int toplimit, int bottomlimit) {
	if(pos == switchheight){
		liftlocation = 90;
	}
	if(pos == scaleheight){
		liftlocation = 0;
	}
	if(pos == driveheight){
		liftlocation = 270;
	}
	srxLiM->Set(ControlMode::Position, pos);
	if(!limit2->Get()){
		srxLiM->GetSensorCollection().SetQuadraturePosition(-toplimit, 10); //Set to Top - 10ms Allowed Time
	}
//	if(!limit->Get()){
//		srxLiM->GetSensorCollection().SetQuadraturePosition(bottomlimit, 10); //Set to Zero - 10ms Allowed Time
//	}
	autostep++;

}

void StartTimer(){
}

