#include <iostream>
#include <string>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <climber.h>
#include <drive.h>
#include <intake.h>
#include <lift.h>
#include <autonomous.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <PowerDistributionPanel.h>
#include <Robot.h>
#include "Timer.h"


int autostep = 0;
std::string gameData;
int autonum; // What the robot is doing
int location; // Where the robot is starting
double autodelay = 0;
double timersec = 0;

class Robot : public frc::IterativeRobot {

public:
	Robot() {
	this->driveManager = new DriveManager();
	this->climberManager = new ClimberManager();
	this->liftManager = new LiftManager();
	this->intakeManager = new IntakeManager();
	this->autoManager = new AutoManager(this->liftManager, this->driveManager, this->intakeManager);

	timer = new Timer();
	}
private:
	DriveManager *driveManager;
	ClimberManager *climberManager;
	LiftManager *liftManager;
	IntakeManager *intakeManager;
	AutoManager *autoManager;
	Timer *timer;

	frc::Joystick stick { 0 };
//	frc::XboxController xbox { 1 };
	//double test;
//	frc::PowerDistributionPanel pdp;





	void RobotInit() {
	}


	void AutonomousInit() override {
		timer->Start();
		this->driveManager->ResetSensors();
		this->driveManager->setCoast();
		//this->liftManager->Lift(scaleheight, switchheight, driveheight); //Setup Talon PID Loop Just incase and to Ensure everything is new

		//frc::SmartDashboard::PutNumber("AutoDelay", 0);
		//frc::SmartDashboard::PutNumber("AutoNumber", 0);
		//frc::SmartDashboard::PutNumber("AutoLocation", 0);


	}

	void AutonomousPeriodic() {
		autodelay = frc::SmartDashboard::GetNumber("AutoDelay", 0);
		autonum = frc::SmartDashboard::GetNumber("AutoNumber", 0);
		location = frc::SmartDashboard::GetNumber("AutoLocation", 0);
		frc::SmartDashboard::PutNumber("AutoNumberResult",autonum);
		frc::SmartDashboard::PutNumber("locationResult", location);

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		frc::SmartDashboard::PutString("Field Data", gameData);

		//this->driveManager->Drive(0.75, ToSwitch);
		frc::SmartDashboard::PutNumber("AutoStep", autostep);
		if(autodelay <= 0){
			timer->Stop();
			timer->Reset();
		}
		timersec = timer->Get();
		if(timersec >= autodelay || autodelay == 0){
			if(autonum == 4){
				this->autoManager->StraightLine();
			}
			if(location == 3 && autonum == 1){
				if(gameData[0] == 'R'){//look for first character only - Hope for Right
					this->autoManager->SwitchRight();
				}
				else{
					this->autoManager->StraightLine();
				}
			}
			if(location == 1 && autonum == 1){
				if(gameData[0] == 'L'){//look for first character only - Hope for Left
					this->autoManager->SwitchLeft(); //Score on the Left when starting on the Left
				}
				else{
					this->autoManager->StraightLine(); //Just Drive Straight
				}
			}
			if(location == 2 && autonum == 1){
				if(gameData[0] == 'L'){//look for first character only
					this->autoManager->CenterLeft(); //Score on the Left when starting Center
				}
				else{//Not Left Do Right
					this->autoManager->CenterRight(); // Score on the Right when starting Center
				}
			}
		}
	}

	void TeleopInit() {
		this->driveManager->ResetSensors();

		this->driveManager->setCoast();
	}

	void TeleopPeriodic() {
		//test = frc::SmartDashboard::GetNumber("batterySet", 0);
		//frc::SmartDashboard::PutNumber("battery#",test);
	//	frc::SmartDashboard::PutNumber("battery voltage",pdp.GetVoltage());

		//creates a arcade drive on talons 1-4 using joystick axies 1-2
		this->driveManager->driveTrain();

		//has a climber on talon 9 controlled with button 11
		this->climberManager->Climber();

		//has a intake on talons 7,8 controlled with button 3 (in) and button 4 (out)
		this->intakeManager->Intake();

		//has a lift with talon 10 controlled with button 5 (up) and 6 (down)
		this->liftManager->Lift(scaleheight, switchheight, driveheight);

		frc::SmartDashboard::PutNumber("Team Number", testglobal);
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
