/*
 * drive.cpp
 *
 *  Created on: Jan 24, 2018
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

	this->stick = new Joystick{ 0 };

	rightStickValue = new double;
	leftStickValue = new double;
	vel1 = new double;
	vel2 = new double;

//using namespace std;

// Constructor - The default values are specified i



}

void DriveManager::driveTrain() {
/*	if (srx1->GetSensorCollection().GetQuadratureVelocity() > -srx2->GetSensorCollection().GetQuadratureVelocity() + 40) {
		frc::SmartDashboard::PutNumber("speed problem",1);
	}
	else {
		frc::SmartDashboard::PutNumber("speed problem",0);
	} */

			if (stick->GetRawButton(1) and !stick->GetRawButton(2)) {
				*rightStickValue = stick->GetRawAxis(1) * 0.5;
				*leftStickValue = stick->GetRawAxis(2) * 0.5;
			}
			else if (!stick->GetRawButton(1)) {
				*rightStickValue = stick->GetRawAxis(1);
			}
			else if (stick->GetRawButton(1) and stick->GetRawButton(2)) {
				*rightStickValue = stick->GetRawAxis(1) * 0.5;
			}

			if (stick->GetRawButton(2)) {
				leftStickValue = 0;
			}
			else if (!stick->GetRawButton(2) and !stick->GetRawButton(1)) {
				*leftStickValue = stick->GetRawAxis(2);
			}

	m_robotDrive->ArcadeDrive(-*rightStickValue, *leftStickValue);
	m_robotDrive2->ArcadeDrive(-*rightStickValue, *leftStickValue);
	m_robotDrive3->ArcadeDrive(-*rightStickValue, *leftStickValue);

	*vel1 = srx1->GetSensorCollection().GetQuadratureVelocity();
	*vel2 = srx2->GetSensorCollection().GetQuadratureVelocity();

	frc::SmartDashboard::PutNumber("velocity1",*vel1);
	frc::SmartDashboard::PutNumber("velocity2",*vel2);

/*	double m1 = srx1->Get();
	double m2 = srx12->Get();
	double m3 = srx13->Get();
	double m4 = srx2->Get();
	double m5 = srx21->Get();
	double m6 = srx22->Get();

	double c1 = srx1->GetOutputCurrent();
	double c2 = srx12->GetOutputCurrent();
	double c3 = srx13->GetOutputCurrent();
	double c4 = srx2->GetOutputCurrent();
	double c5 = srx21->GetOutputCurrent();
	double c6 = srx22->GetOutputCurrent();

	frc::SmartDashboard::PutNumber("m1",m1);
	frc::SmartDashboard::PutNumber("m2",m2);
	frc::SmartDashboard::PutNumber("m3",m3);
	frc::SmartDashboard::PutNumber("m4",m4);
	frc::SmartDashboard::PutNumber("m5",m5);
	frc::SmartDashboard::PutNumber("m6",m6);

	frc::SmartDashboard::PutNumber("c1",c1);
	frc::SmartDashboard::PutNumber("c2",c2);
	frc::SmartDashboard::PutNumber("c3",c3);
	frc::SmartDashboard::PutNumber("c4",c4);
	frc::SmartDashboard::PutNumber("c5",c5);
	frc::SmartDashboard::PutNumber("c6",c6); */
}




