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
#include "AHRS.h"
#include <SPI.h>
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
	xbox = new XboxController { 1 };

	rightStickValue = new double;
	leftStickValue = new double;
	vel1 = new double;
	vel2 = new double;
	dis = new double;
	dis2 = new double;
	init = new int;
	one = new int;

	ahrs = new AHRS(SPI::Port::kMXP);
	ahrs->Reset();

}

double want = 0;
double gyro = 0;
double k = -0.2;
double x = 0;
double z = 0;

double m1;
double m2;
double m3;
double m4;
double m5;
double m6;

double c1;
double c2;
double c3;
double c4;
double c5;
double c6;

double encRot;
double encRot2;

double distance;
double distance2;

void DriveManager::driveTrain() {
	if (stick->GetRawAxis(1) < 0.05 and stick->GetRawAxis(1) > -0.05) {
		x = 0;
	}
	else {
		x = stick->GetRawAxis(1);
	}

	if (stick->GetRawAxis(2) < 0.05 and stick->GetRawAxis(2) > -0.05) {
		z = 0;
	}
	else {
		z = stick->GetRawAxis(2);
	}


			if (stick->GetRawButton(1) and !stick->GetRawButton(2)) {
				x = x * 0.5;
				z = z * 0.5;
			}
			else if (stick->GetRawButton(1) and stick->GetRawButton(2)) {
				x = x * 0.5;
			}

			if (stick->GetRawButton(2)) {
				z = 0;
			}


	*vel1 = srx1->GetSensorCollection().GetQuadratureVelocity();
	*vel2 = -srx2->GetSensorCollection().GetQuadratureVelocity();
	*dis = srx1->GetSensorCollection().GetQuadraturePosition();
	*dis2 = -srx2->GetSensorCollection().GetQuadraturePosition();

	frc::SmartDashboard::PutNumber("velocity1",*vel1);
	frc::SmartDashboard::PutNumber("velocity2",*vel2);
	frc::SmartDashboard::PutNumber("distance",*dis);
	frc::SmartDashboard::PutNumber("distance2",*dis2);

	if (stick->GetRawButton(5)) {
		srx1->GetSensorCollection().SetQuadraturePosition(0,4);
		srx2->GetSensorCollection().SetQuadraturePosition(0,4);
	}
	//4000 = one rotation ?4096
	//diameter = 3.94 in
	//circumfrece = 24.7432

	encRot = (1.0 * srx1->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations",encRot);
	distance = (encRot * 12.566);
	frc::SmartDashboard::PutNumber("distanceInches",distance);

	encRot2 = (1.0 * -srx2->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations2",encRot2);
	distance2 = (encRot2 * 12.566);
	frc::SmartDashboard::PutNumber("distanceInches2",distance2);

	m1 = srx1->Get();
	m2 = srx12->Get();
	m3 = srx13->Get();
	m4 = srx2->Get();
	m5 = srx21->Get();
	m6 = srx22->Get();

	c1 = srx1->GetOutputCurrent();
	c2 = srx12->GetOutputCurrent();
	c3 = srx13->GetOutputCurrent();
	c4 = srx2->GetOutputCurrent();
	c5 = srx21->GetOutputCurrent();
	c6 = srx22->GetOutputCurrent();

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
	frc::SmartDashboard::PutNumber("c6",c6);

	//clockwise is positive
	gyro = ahrs->GetYaw();
	frc::SmartDashboard::PutNumber("gAngle",gyro);

	if (stick->GetRawButton(6)) {
		ahrs->Reset();
		want = gyro;
	}

	if (stick->GetRawButton(2)) {
		if (z == 0) {
			z = ((gyro - want) * k);
		}
		else {
			//want = gyro;
		}
	}
	frc::SmartDashboard::PutNumber("want",want);

	m_robotDrive->ArcadeDrive(-x, z);
	m_robotDrive2->ArcadeDrive(-x, z);
	m_robotDrive3->ArcadeDrive(-x, z);
}




