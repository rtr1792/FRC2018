#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <segments/climber.h>
#include <segments/drive.h>
#include <segments/intake.h>
#include <segments/lift.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>

class Robot : public frc::IterativeRobot {

public:
	Robot() {
	this->driveManager = new DriveManager();
	this->climberManager = new ClimberManager();
	this->liftManager = new LiftManager();
	this->intakeManager = new IntakeManager();
	}
private:
	DriveManager *driveManager;
	ClimberManager *climberManager;
	LiftManager *liftManager;
	IntakeManager *intakeManager;

	frc::Joystick stick { 0 };
//	frc::XboxController xbox { 1 };

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}


	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData[0] == 'L')
		{
			frc::SmartDashboard::PutNumber("side",1);
		}

		else {
			frc::SmartDashboard::PutNumber("side",2);
		}

//		this->liftManager->Liftmove(1000);
//		this->intakeManager->Intakemove(0.5);
	}

	void TeleopInit() {}

	void TeleopPeriodic() {
		//creates a arcade drive on talons 1-4 using joystick axies 1-2
		this->driveManager->driveTrain();

		//has a climber on talon 9 controlled with button 11
		this->climberManager->Climber();

		//has a intake on talons 7,8 controlled with button 3 (in) and button 4 (out)
		this->intakeManager->Intake();

		//has a lift with talon 10 controlled with button 5 (up) and 6 (down)
		this->liftManager->Lift();

/*		bool x1 = xbox.GetRawButton(1);
		double x2 = xbox.GetRawAxis(5);

		frc::SmartDashboard::PutNumber("x1",x1);
		frc::SmartDashboard::PutNumber("x2",x2); */
}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
