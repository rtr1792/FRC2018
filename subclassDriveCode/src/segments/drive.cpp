/*
 * drive.cpp
 *
 *  Created on: Jan 24, 2018
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
#include "drive.h"
#include <iostream>
#include <Encoder.h>
//n the declaration

DriveManager::DriveManager() {
	srx1 = new WPI_TalonSRX(1);
	srx12 = new WPI_TalonSRX(2);
	srx13 = new WPI_TalonSRX(3);

	srx2 = new WPI_TalonSRX(4);
	srx21 = new WPI_TalonSRX(5);
	srx22 = new WPI_TalonSRX(6);

	m_robotDrive = new DifferentialDrive(*srx1, *srx2);
	m_robotDrive2 = new DifferentialDrive(*srx12, *srx21);
	m_robotDrive3 = new DifferentialDrive(*srx13, *srx22);
	m_robotDrive->SetSafetyEnabled(false);
	m_robotDrive2->SetSafetyEnabled(false);
	m_robotDrive3->SetSafetyEnabled(false);

	//this->leftStickValue;
	//this->rightStickValue;

	this->stick = new Joystick{ 0 };
	//frc::Joystick stick { 0 };

	//double rightMotor;
	//double leftMotor;
	//double rightStickValue;
	//double leftStickValue;
using namespace std;

// Constructor - The default values are specified i



}

void DriveManager::driveTrain() {
			if (stick->GetRawButton(1) and !stick->GetRawButton(2)) {
				rightStickValue = stick->GetRawAxis(1) * 0.5;
				leftStickValue = stick->GetRawAxis(2) * 0.5;
			}
			else if (!stick->GetRawButton(1)) {
				rightStickValue = stick->GetRawAxis(1);
			}
			else if (stick->GetRawButton(1) and stick->GetRawButton(2)) {
				rightStickValue = stick->GetRawAxis(1) * 0.5;
			}

			if (stick->GetRawButton(2)) {
				leftStickValue = 0;
			}
			else if (!stick->GetRawButton(2) and !stick->GetRawButton(1)) {
				leftStickValue = stick->GetRawAxis(2);
			}

	m_robotDrive->ArcadeDrive(-rightStickValue, leftStickValue);
	m_robotDrive2->ArcadeDrive(-rightStickValue, leftStickValue);
	m_robotDrive3->ArcadeDrive(-rightStickValue, leftStickValue);


	vel1 = srx1->GetSensorCollection().GetQuadratureVelocity();
	vel2 = srx2->GetSensorCollection().GetQuadratureVelocity();

	frc::SmartDashboard::PutNumber("velocity1",vel1);
	frc::SmartDashboard::PutNumber("velocity2",vel2);
}




