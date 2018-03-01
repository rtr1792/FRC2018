/*
 * autonomous.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include "autonomous.h"
#include "math.h"
#include <Timer.h>
#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <Encoder.h>
#include "AHRS.h"
#include <SPI.h>
#include <Robot.h>

//Crosses the baseline, nothing more
AutoManager::AutoManager(LiftManager *lift, DriveManager *drive, IntakeManager *intake) {
	this->liftManager = lift;
	this->driveManager = drive;
	this->intakeManager = intake;
}
//SwitchRight
void AutoManager::SwitchRight() {
	//Switch Right
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, ToSwitch);  //find dist to switch
				this->intakeManager->Intakemove(0, true);
			break;
		case 2 : this->driveManager->Turn(-90); // Negative to Turn Left
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4: this->driveManager->Drive(SecondDriveSpeed, NexttoSwitchForward); //find dist to switch from pt
				this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->intakeManager->Intakemove(-1, false);
			break;
	}
}

//Switch Left
void AutoManager::SwitchLeft() {
	//Switch Left
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, ToSwitch);  //find dist to switch
				this->intakeManager->Intakemove(0, true);
			break;
		case 2 : this->driveManager->Turn(90); // Positive to turn Right
				this->intakeManager->Intakemove(0, true);
			break;
		case 3: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 4: this->driveManager->Drive(SecondDriveSpeed, NexttoSwitchForward); //find dist to switch from pt
				this->intakeManager->Intakemove(0, true);
			break;
		case 5: this->intakeManager->Intakemove(-1, false);
			break;
	}
}
//Score on the Right Side of the Switch
void AutoManager::CenterRight(){
	//Switch Center
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);                                     // do things from same thing above but reverse also DIFFERENT ANGLES!!!!!!!!!!!!!!!!!!!
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, ToSwitch); // negative to Turn left
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->intakeManager->Intakemove(-1, false);
			break;
	}
}
//Score on the Left Side of the Switch
void AutoManager::CenterLeft(){
	//Switch Center
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Turn(CenterDegreeTurn); // Postive to Turn Right
				this->intakeManager->Intakemove(0, true);
			break;
		case 2: this->driveManager->ResetSensors();
				this->driveManager->FindStartEnc();
			break;
		case 3 : this->driveManager->Drive(FirstDriveSpeed, CenterDriveDistafterTurn);
				 this->intakeManager->Intakemove(0, true);
			break;
		case 4: this->intakeManager->Intakemove(-1, false);
			break;
	}
}

//Straight Line Drive Auto Line
void AutoManager::StraightLine() {
	//Straight Line Drive Auto Line
	switch(autostep){
		case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
				this->intakeManager->Intakemove(0, true);
			break;
		case 1 : this->driveManager->Drive(FirstDriveSpeed, ToSwitch);
				this->intakeManager->Intakemove(0, true);
			break;
	}
}



