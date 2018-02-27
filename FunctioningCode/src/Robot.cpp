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
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <PowerDistributionPanel.h>
#include <Robot.h>

int autostep = 0;
std::string gameData;
int autonum; // What the robot is doing
int location; // Where the robot is starting

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
	//double test;
//	frc::PowerDistributionPanel pdp;





	void RobotInit() {

	}


	void AutonomousInit() override {
		this->driveManager->ResetSensors();
		//frc::SmartDashboard::PutNumber("auto",0);
		this->driveManager->setCoast();
	}

	void AutonomousPeriodic() {
		autonum = frc::SmartDashboard::GetNumber("auto", 0);
		location = frc::SmartDashboard::GetNumber("Location", 0);
		frc::SmartDashboard::PutNumber("autoResult",autonum);
		frc::SmartDashboard::PutNumber("locationResult", location);

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		frc::SmartDashboard::PutString("Field Data", gameData);

		//this->driveManager->Drive(0.75, autoLine);
		frc::SmartDashboard::PutNumber("AutoStep", autostep);
		if(autonum == 4){
			//Straight Line Drive Auto Line
			switch(autostep){
				case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
						this->intakeManager->Intakemove(0, true);
					break;
				case 1 : this->driveManager->Drive(0.65, autoLine);
						this->intakeManager->Intakemove(0, true);
					break;
			}
		}
		if(location == 3 && autonum == 1){
			if(gameData == "RRR" || gameData == "RRL" || gameData == "RLL" || gameData == "RLR"){   //look for first character only
				//Switch Right
				switch(autostep){
				case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
						this->intakeManager->Intakemove(0, true);
					break;
				case 1 : this->driveManager->Drive(0.65, autoLine);  //find dist to switch
						this->intakeManager->Intakemove(0, true);
					break;
				case 2 : this->driveManager->Turn(-90); // Negative to Turn Left
						this->intakeManager->Intakemove(0, true);
					break;
				case 3: this->driveManager->ResetSensors();
						this->driveManager->Drive(0.3, 26); //find dist to switch from pt
						this->intakeManager->Intakemove(0, true);
					break;
				case 4: this->intakeManager->Intakemove(-1, false);
					break;
				}
			}
			else{
				//Straight Line Drive Auto Line
				switch(autostep){
					case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
							this->intakeManager->Intakemove(0, true);
						break;
					case 1 : this->driveManager->Drive(0.65, autoLine);
							this->intakeManager->Intakemove(0, true);
						break;
				}
			}
		}
		if(location == 1 && autonum == 1){
			if(gameData == "LRR" || gameData == "LRL" || gameData == "LLL" || gameData == "LLR"){
				//Switch Left
				switch(autostep){
				case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
						this->intakeManager->Intakemove(0, true);
					break;
				case 1 : this->driveManager->Drive(0.65, autoLine); //redetermine auto line
						this->intakeManager->Intakemove(0, true);
					break;
				case 2 : this->driveManager->Turn(90); // Postive to Turn Right
						this->intakeManager->Intakemove(0, true);
					break;
				case 3: this->driveManager->ResetSensors();
						this->driveManager->Drive(0.3, 26); //determine dist
						this->intakeManager->Intakemove(0, true);
					break;
				case 4: this->intakeManager->Intakemove(-1, false);
					break;
				}
			}
			else{
				//Straight Line Drive Auto Line
				switch(autostep){
					case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
							this->intakeManager->Intakemove(0, true);
						break;
					case 1 : this->driveManager->Drive(0.65, autoLine);
							this->intakeManager->Intakemove(0, true);
						break;
				}
			}
		}
		if(location == 2 && autonum == 1){
			if(gameData == "LRR" || gameData == "LRL" || gameData == "LLL" || gameData == "LLR"){
				//Switch Center
				switch(autostep){
				case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
						this->intakeManager->Intakemove(0, true);
					break;
				case 1 : this->driveManager->Turn(45); // Postive to Turn Right                   //DO TRIG AND THINGS!!!!!!  hypotenuse and such  !!!!!!!!!!!!!
						this->intakeManager->Intakemove(0, true);
					break;
				case 2 : this->driveManager->Drive(0.65, autoLine);
						 this->intakeManager->Intakemove(0, true);
					break;

					// add turn (90-angle from case 1)in opposite direction of case 1

				case 3: this->driveManager->ResetSensors();
						this->driveManager->Drive(0.3, 26);
						this->intakeManager->Intakemove(0, true);
					break;
				case 4: this->intakeManager->Intakemove(-1, false);
					break;
				}
			}
			else{
				//Switch Center
				switch(autostep){
				case 0: this->liftManager->Liftmove(switchheight, scaleheight, 0);
						this->intakeManager->Intakemove(0, true);                                     // do things from same thing above but reverse also DIFFERENT ANGLES!!!!!!!!!!!!!!!!!!!
					break;
				case 1 : this->driveManager->Turn(-45); // negative to Turn left
						this->intakeManager->Intakemove(0, true);
					break;
				case 2 : this->driveManager->Drive(0.65, autoLine); /////hypotenuse MAAATH
						 this->intakeManager->Intakemove(0, true);
					break;
				case 3: this->driveManager->ResetSensors();
						this->driveManager->Drive(0.3, 26);
						this->intakeManager->Intakemove(0, true);
					break;
				case 4: this->intakeManager->Intakemove(-1, false);
					break;
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
