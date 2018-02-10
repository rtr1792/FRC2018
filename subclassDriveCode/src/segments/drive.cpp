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

	joystickDeadBandX = new double;
	joystickDeadBandZ = new double;

	joystickDeadBandX = 0;
	joystickDeadBandZ = 0;
}

void DriveManager::driveTrain() {
	float deadZoneThreshold = 0.3;

	if((fabs(stick->GetRawAxis(0)) < deadZoneThreshold) or stick->GetRawButton(12))
	{
		*joystickDeadBandX = 0;
	}
	//Otherwise set to joystick value
	else
	{
		*joystickDeadBandX = stick->GetRawAxis(0);
	}

	//Repeat of above for Z
	if((fabs(stick->GetRawAxis(2)) < 0.25))
	{
		*joystickDeadBandZ = 0;
	}
	else
	{
		*joystickDeadBandZ = -stick->GetRawAxis(2);
	}

/*

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
				*leftStickValue = 0;
			}
			else if (!stick->GetRawButton(2) and !stick->GetRawButton(1)) {
				*leftStickValue = stick->GetRawAxis(2);
			} */


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

	double encRot = (1.0 * srx1->GetSensorCollection().GetQuadraturePosition() / 4000);
	frc::SmartDashboard::PutNumber("encRotations",encRot);
	double distance = (encRot * 24.7432);
	frc::SmartDashboard::PutNumber("distanceInches",distance);

	double encRot2 = (1.0 * -srx2->GetSensorCollection().GetQuadraturePosition() / 4000);
	frc::SmartDashboard::PutNumber("encRotations2",encRot2);
	double distance2 = (encRot2 * 24.7432);
	frc::SmartDashboard::PutNumber("distanceInches2",distance2);

	double m1 = srx1->Get();
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
	frc::SmartDashboard::PutNumber("c6",c6);

	//clockwise is positive
	double gyro = ahrs->GetYaw();
	double want = 0;
	double k = -0.09;
	frc::SmartDashboard::PutNumber("gAngle",gyro);

	if (stick->GetRawButton(6)) {
		ahrs->Reset();
		want = gyro;
	}

	if (stick->GetRawAxis(2) == 0) {
		*leftStickValue = ((gyro - want) * k);
	}
	else {
		*leftStickValue = stick->GetRawAxis(2);
		want = gyro;
	}

	m_robotDrive->ArcadeDrive(-*joystickDeadBandX, *joystickDeadBandZ);
	m_robotDrive2->ArcadeDrive(-*joystickDeadBandX, *joystickDeadBandZ);
	m_robotDrive3->ArcadeDrive(-*joystickDeadBandX, *joystickDeadBandZ);
}




