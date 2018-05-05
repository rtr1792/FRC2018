/*
 * drive.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: RTR
 */

//buttons 1,2,5,6,7,8,9,10

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
#include <Robot.h>
#include "Timer.h"
//n the declaration
bool VelocityControl = false;
double left = 0;
double right = 0;
int kTimeoutMs = 10;
bool CurrentControl = false;

DriveManager::DriveManager() {
	turnTimer = new Timer();
	delayTimer= new Timer();

	srxDtLm = new WPI_TalonSRX(1);
	srxDtLs1 = new WPI_TalonSRX(2);
	srxDtLs2 = new WPI_TalonSRX(3);

	srxDtRm = new WPI_TalonSRX(4);
	srxDtRs1 = new WPI_TalonSRX(5);
	srxDtRs2 = new WPI_TalonSRX(6);

	if(!VelocityControl){
		m_robotDrive = new DifferentialDrive(*srxDtLm, *srxDtRm);
		m_robotDrive->SetSafetyEnabled(true);
	}
	else{
		double kPIDLoopIdx = 0;
		bool Inverted = true;

		double Fgain = 0.1532;
		double Pgain = 0.0;
		double Igain = 0.0;
		double Dgain = 0.0;

		//srxDtLm
		srxDtLm->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		srxDtLm->SetSensorPhase(true);
		srxDtLm->SetInverted(Inverted);
		srxDtLm->ConfigAllowableClosedloopError(0, 0, kTimeoutMs);


		/* set the peak and nominal outputs, 12V means full */
		srxDtLm->ConfigNominalOutputForward(0, kTimeoutMs);
		srxDtLm->ConfigNominalOutputReverse(0, kTimeoutMs);
		srxDtLm->ConfigPeakOutputForward(12, kTimeoutMs);
		srxDtLm->ConfigPeakOutputReverse(-12 , kTimeoutMs);

		/* set closed loop gains in slot0 */
		srxDtLm->Config_kF(kPIDLoopIdx, Fgain, kTimeoutMs);
		srxDtLm->Config_kP(kPIDLoopIdx, Pgain, kTimeoutMs);
		srxDtLm->Config_kI(kPIDLoopIdx, Igain, kTimeoutMs);
		srxDtLm->Config_kD(kPIDLoopIdx, Dgain, kTimeoutMs);

		//srxDtRm
		srxDtRm->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		srxDtRm->SetSensorPhase(true);
		srxDtRm->SetInverted(Inverted);
		srxDtRm->ConfigAllowableClosedloopError(0, 0, kTimeoutMs);


		/* set the peak and nominal outputs, 12V means full */
		srxDtRm->ConfigNominalOutputForward(0, kTimeoutMs);
		srxDtRm->ConfigNominalOutputReverse(0, kTimeoutMs);
		srxDtRm->ConfigPeakOutputForward(12, kTimeoutMs);
		srxDtRm->ConfigPeakOutputReverse(-12 , kTimeoutMs);

		/* set closed loop gains in slot0 */
		srxDtRm->Config_kF(kPIDLoopIdx, Fgain, kTimeoutMs);
		srxDtRm->Config_kP(kPIDLoopIdx, Pgain, kTimeoutMs);
		srxDtRm->Config_kI(kPIDLoopIdx, Igain, kTimeoutMs);
		srxDtRm->Config_kD(kPIDLoopIdx, Dgain, kTimeoutMs);
	}


	srxDtLs1->Set(ControlMode::Follower, 1);
	srxDtLs2->Set(ControlMode::Follower, 1);
	srxDtRs1->Set(ControlMode::Follower, 4);
	srxDtRs2->Set(ControlMode::Follower, 4);

	//Left Current Enabling
	srxDtLm->EnableCurrentLimit(CurrentControl);
	srxDtLs1->EnableCurrentLimit(CurrentControl);
	srxDtLs2->EnableCurrentLimit(CurrentControl);

	//Right Current Enabling
	srxDtRm->EnableCurrentLimit(CurrentControl);
	srxDtRs1->EnableCurrentLimit(CurrentControl);
	srxDtRs2->EnableCurrentLimit(CurrentControl);

	//Left Continuous Current Limits
	srxDtLm->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srxDtLs1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srxDtLs2->ConfigContinuousCurrentLimit(40, kTimeoutMs);

	//Right Continuous Current Limits
	srxDtRm->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srxDtRs1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
	srxDtRs2->ConfigContinuousCurrentLimit(40, kTimeoutMs);

	this->stick = new Joystick{ 0 };
	xbox = new XboxController { 1 };

	rightStickValue = new double;
	leftStickValue = new double;
	vel1 = new double;
	vel2 = new double;
	dist = new double;
	dist2 = new double;
	init = new int;
	one = new int;

	//Adding Try Catch to Avoid Locking up robot
	try{
		ahrs = new AHRS(SPI::Port::kMXP);
	}
	catch(std::exception ex){
		std::string err_string = "Error initalizing navX-MXP: ";
		err_string += ex.what();
		DriverStation::ReportError(err_string.c_str());
	}
	ahrs->Reset();

	srxDtLm->GetSensorCollection().SetQuadraturePosition(0,10);
	srxDtRm->GetSensorCollection().SetQuadraturePosition(0,10);
}
int delayLoop = 0;  //loop counter for delay/skip function
int loopCount = 0;	//loop counter for the auto turn boost
double boost = 0;	//starting value for the auto boost

double want = 0;
double gyro = 0;
double turnk = -0.3;
double tiltk = -0.01;
double x = 0;
double z = 0;

double driveLeftMasterPercentVoltage;
double driveLeftSlaveOnePercentVoltage;
double driveLeftSlaveTwoPercentVoltage;
double driveRightMasterPercentVoltage;
double driveRightSlaveOnePercentVoltage;
double driveRightSlaveTwoPercentVoltage;

//Dt=drive train L=left m=master s=slave
double DtLmCurrent;
double DtLs1Current;
double DtLs2Current;
double DtRmCurrent;
double DtRs1Current;
double DtRs2Current;

double encRot;
double encRot2;

double LeftDist;
double RightDist;

double LeftEncLast = 0.0;
double RightEncLast = 0.0;

bool gyroRefresh;

void DriveManager::driveTrain() {
	gyroRefresh = ahrs->IsConnected();
	frc::SmartDashboard::PutBoolean("gyroConnected",gyroRefresh);

//deadband
	if (stick->GetRawAxis(1) < 0.05 and stick->GetRawAxis(1) > -0.05) {
		x = 0;
	}
	else if(stick->GetRawButton(4)){
		x = stick->GetRawAxis(1);
	}
	else{
		x = stick->GetRawAxis(1) * 0.85;
	}

	if (stick->GetRawAxis(2) < 0.05 and stick->GetRawAxis(2) > -0.05) {
		z = 0;
	}
	else if(stick->GetRawButton(4)){
		z = stick->GetRawAxis(2)*0.75;
	}
	else{
		z = stick->GetRawAxis(2) * 0.75;
	}


	//creep and turnlock
			if (stick->GetRawButton(1) and !stick->GetRawButton(2)) {
				x = stick->GetRawAxis(1) * 0.5;
				z = stick->GetRawAxis(2) * 0.65;
			}
			else if (stick->GetRawButton(1) and stick->GetRawButton(2)) {
				x = stick->GetRawAxis(1) * 0.5;
				z = 0;
			}

			if (stick->GetRawButton(2)) {
				z = 0;
			}


			//gets encoder values
	*vel1 = srxDtLm->GetSensorCollection().GetQuadratureVelocity();
	*vel2 = -srxDtRm->GetSensorCollection().GetQuadratureVelocity();
	*dist = srxDtLm->GetSensorCollection().GetQuadraturePosition();
	*dist2 = -srxDtRm->GetSensorCollection().GetQuadraturePosition();

	frc::SmartDashboard::PutNumber("velocity1",*vel1);
	frc::SmartDashboard::PutNumber("velocity2",*vel2);
	frc::SmartDashboard::PutNumber("LeftDist",*dist);
	frc::SmartDashboard::PutNumber("RightDist",*dist2);

	//resets encoders
	if (stick->GetRawButton(5)) {
		srxDtLm->GetSensorCollection().SetQuadraturePosition(0,4);
		srxDtRm->GetSensorCollection().SetQuadraturePosition(0,4);
	}
	//4000 = one rotation ?4096
	//diameter = 3.94 in
	//circumfrece = 24.7432

	//converts encoder value to inches
	encRot = (1.0 * srxDtLm->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations",encRot);
	LeftDist = (encRot * 12.566);
	frc::SmartDashboard::PutNumber("distanceInches",LeftDist);

	encRot2 = (1.0 * -srxDtRm->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations2",encRot2);
	RightDist = (encRot2 * 12.566);
	frc::SmartDashboard::PutNumber("distanceInches2",RightDist);

	driveLeftMasterPercentVoltage = srxDtLm->Get();
	driveLeftSlaveOnePercentVoltage = srxDtLs1->Get();
	driveLeftSlaveTwoPercentVoltage = srxDtLs2->Get();
	driveRightMasterPercentVoltage = srxDtRm->Get();
	driveRightSlaveOnePercentVoltage = srxDtRs1->Get();
	driveRightSlaveTwoPercentVoltage = srxDtRs2->Get();

	DtLmCurrent = srxDtLm->GetOutputCurrent();
	DtLs1Current = srxDtLs1->GetOutputCurrent();
	DtLs2Current = srxDtLs2->GetOutputCurrent();
	DtRmCurrent = srxDtRm->GetOutputCurrent();
	DtRs1Current = srxDtRs1->GetOutputCurrent();
	DtRs2Current = srxDtRs2->GetOutputCurrent();

	frc::SmartDashboard::PutNumber("driveLeftMasterPercentVoltage",driveLeftMasterPercentVoltage);
	frc::SmartDashboard::PutNumber("driveLeftSlaveOnePercentVoltage",driveLeftSlaveOnePercentVoltage);
	frc::SmartDashboard::PutNumber("driveLeftSlaveTwoPercentVoltage",driveLeftSlaveTwoPercentVoltage);
	frc::SmartDashboard::PutNumber("driveRightMasterPercentVoltage",driveRightMasterPercentVoltage);
	frc::SmartDashboard::PutNumber("driveRightSlaveOnePercentVoltage",driveRightSlaveOnePercentVoltage);
	frc::SmartDashboard::PutNumber("driveRightSlaveTwoPercentVoltage",driveRightSlaveTwoPercentVoltage);

	frc::SmartDashboard::PutNumber("DtLmCurrent",DtLmCurrent);
	frc::SmartDashboard::PutNumber("DtLmCurrent",DtLmCurrent);
	frc::SmartDashboard::PutNumber("DtLs2Current",DtLs2Current);
	frc::SmartDashboard::PutNumber("DtRmCurrent",DtRmCurrent);
	frc::SmartDashboard::PutNumber("DtRs1Current",DtRs1Current);
	frc::SmartDashboard::PutNumber("DtRs2Current",DtRs2Current);

	//gyro values
	//clockwise is positive
	double Yaw = ahrs->GetYaw();
	double Pitch = ahrs->GetPitch();
	double Roll = ahrs->GetRoll();
	gyro = ahrs->GetAngle();
	frc::SmartDashboard::PutNumber("Yaw", Yaw);
	//frc::SmartDashboard::PutString("Firmware", ahrs->GetFirmwareVersion());
	frc::SmartDashboard::PutNumber("Pitch", Pitch);
	frc::SmartDashboard::PutNumber("Roll", Roll);
	frc::SmartDashboard::PutNumber("Angle", gyro);

	//resets gyro
	if (stick->GetRawButton(6)) {
		ahrs->Reset();
		want = gyro;
	}

	//strait drive
	if (stick->GetRawButton(2)) {
		if (z == 0) {
			z = ((gyro - want) * turnk);
		}
		else {
			want = gyro;
		}
	}
	frc::SmartDashboard::PutNumber("want",want);

	/*
	if(Pitch > 15 || -15 > Pitch){
		x = x + (gyro) * tiltk;
	}
	*/

	if(VelocityControl){
		left = x + z;
		right = x - z;
		left = ((left*(5330/5.45)*4096)/600); //Percentage * 5330RPM of CIM / GearRatio * 4096units in 1 rotation / 600 to units per ms
		right = ((right*(5330/5.45)*4096)/600); //Percentage * 5330RPM of CIM / GearRatio * 4096units in 1 rotation / 600 to units per ms
		srxDtLm->Set(ControlMode::Velocity, left);
		srxDtRm->Set(ControlMode::Velocity, right);
		frc::SmartDashboard::PutNumber("Left Side Error", srxDtLm->GetClosedLoopError(0));
		frc::SmartDashboard::PutNumber("Right Side Error", srxDtRm->GetClosedLoopError(0));
	}
	else{
		m_robotDrive->ArcadeDrive(-x, z);
	}
	// New Code of 2/26/2018 To Set Brake or Coast
	//Set to Brake <Sarcasm> (IF NOT OBVIOUS) </Sarcasm>
	if(stick->GetRawButton(8) || stick->GetRawButton(10)){ //Button Redundancy
		//Left Side
		srxDtLm->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
		srxDtLs1->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
		srxDtLs2->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
		//Right Side
		srxDtRm->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
		srxDtRs1->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
		srxDtRs2->SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
	}
	//Set to Coast <Sarcasm> (IF NOT OBVIOUS) </Sarcasm>
	if(stick->GetRawButton(7) || stick->GetRawButton(9)){ //Button Redundancy
			//Left Side
			srxDtLm->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
			srxDtLs1->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
			srxDtLs2->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
			//Right Side
			srxDtRm->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
			srxDtRs1->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
			srxDtRs2->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
	}


}
//drive for auto
void DriveManager::Drive(double speed, double goDistance) {
	double z;
	double turnk = -0.10; //-0.17
	double want = 0;
	gyro = ahrs->GetAngle();

	encRot = (1.0 * -srxDtLm->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations",encRot);
	LeftDist = (encRot * 12.566)-LeftEncLast; //Subrtacting LeftEncLast inorder to get Displacement to account for already driven
	frc::SmartDashboard::PutNumber("distanceInches",LeftDist);

	encRot2 = (1.0 * srxDtRm->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations2",encRot2);
	RightDist = (encRot2 * 12.566) - RightEncLast; //Subtracting RightEncLast inorder to get Displacement to account for already driven
	frc::SmartDashboard::PutNumber("distanceInches2",RightDist);


	z = ((gyro - want) * turnk);
	if ((LeftDist+RightDist)/2 <  goDistance) { //This now measures the average displacement vs the target displacement
		m_robotDrive->ArcadeDrive(speed, z);
		frc::SmartDashboard::PutNumber("motorSpeed",speed);
		frc::SmartDashboard::PutNumber("AutoDistance",goDistance);
	}
	else{
		autostep++;
	}

	driveLeftMasterPercentVoltage = srxDtLm->Get();
	driveLeftSlaveOnePercentVoltage = srxDtLs1->Get();
	driveLeftSlaveTwoPercentVoltage = srxDtLs2->Get();
	driveRightMasterPercentVoltage = srxDtRm->Get();
	driveRightSlaveOnePercentVoltage = srxDtRs1->Get();
	driveRightSlaveTwoPercentVoltage = srxDtRs2->Get();

	frc::SmartDashboard::PutNumber("driveLeftMasterPercentVoltage",driveLeftMasterPercentVoltage);
	frc::SmartDashboard::PutNumber("driveLeftSlaveOnePercentVoltage",driveLeftSlaveOnePercentVoltage);
	frc::SmartDashboard::PutNumber("driveLeftSlaveTwoPercentVoltage",driveLeftSlaveTwoPercentVoltage);
	frc::SmartDashboard::PutNumber("driveRightMasterPercentVoltage",driveRightMasterPercentVoltage);
	frc::SmartDashboard::PutNumber("driveRightSlaveOnePercentVoltage",driveRightSlaveOnePercentVoltage);
	frc::SmartDashboard::PutNumber("driveRightSlaveTwoPercentVoltage",driveRightSlaveTwoPercentVoltage);
}


//End NEW CODE for Brake / Coast
//allows the auto to turn
void DriveManager::Turn(int angle){
	double z;
	double turnk = -0.015; //Bigger Numbers ARE FASTER
	double want = angle;
	//double allowederror = 5;
	gyro = ahrs->GetAngle();
	z = ((gyro - want) * turnk);
	if(fabs(gyro-angle)< 4){  //hardcode 5 as tolerance
		autostep++;
	}

	m_robotDrive->ArcadeDrive(0, z);
	frc::SmartDashboard::PutNumber("Gyro", gyro);
	frc::SmartDashboard::PutNumber("Want", want);
}
void DriveManager::ResetSensors(){
	ahrs->Reset();
	ahrs->ZeroYaw();
}

void DriveManager::setCoast(){
	//Left Side
	srxDtLm->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
	srxDtLs1->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
	srxDtLs2->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
	//Right Side
	srxDtRm->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
	srxDtRs1->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
	srxDtRs2->SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
}
void DriveManager::FindStartEnc(){
	//Cannot be run Co-currently has to be run in its own step
	LeftEncLast = -srxDtLm->GetSensorCollection().GetQuadraturePosition();
	RightEncLast = srxDtRm->GetSensorCollection().GetQuadraturePosition();

	LeftEncLast = (LeftEncLast/4096.0);
	RightEncLast = (RightEncLast/4096.0);

	LeftEncLast = LeftEncLast*12.566;
	RightEncLast = RightEncLast*12.566;

	autostep++; //Cannot be run Co-currently has to be run in its own step
}

//new auto drive should allow for backward drive. Enter -speed and -goDistance for backward movement
void DriveManager::DriveNew(double speed, double goDistance) {
	double z;
	double turnk = -0.10; //-0.17
	double want = 0;
	gyro = ahrs->GetAngle();

	encRot = (1.0 * -srxDtLm->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations",encRot);
	LeftDist = (encRot * 12.566)-LeftEncLast; //Subrtacting LeftEncLast inorder to get Displacement to account for already driven
	frc::SmartDashboard::PutNumber("distanceInches",LeftDist);

	encRot2 = (1.0 * srxDtRm->GetSensorCollection().GetQuadraturePosition() / 4096);
	frc::SmartDashboard::PutNumber("encRotations2",encRot2);
	RightDist = (encRot2 * 12.566) - RightEncLast; //Subtracting RightEncLast inorder to get Displacement to account for already driven
	frc::SmartDashboard::PutNumber("distanceInches2",RightDist);

if (goDistance > 0 or goDistance == 0) {
	z = ((gyro - want) * turnk);
	if ((LeftDist+RightDist)/2 < goDistance) { //This now measures the average displacement vs the target displacement
		m_robotDrive->ArcadeDrive(speed, z);
		frc::SmartDashboard::PutNumber("motorSpeed",speed);
		frc::SmartDashboard::PutNumber("AutoDistance",goDistance);
	}
	else{
		autostep++;
	}
}
else { //the backward drive code
	z = ((gyro - want) * turnk);
	if ((LeftDist+RightDist)/2 > goDistance) { //This now measures the average displacement vs the target displacement
		m_robotDrive->ArcadeDrive(-speed, z);
		frc::SmartDashboard::PutNumber("motorSpeed",speed);
		frc::SmartDashboard::PutNumber("AutoDistance",goDistance);
	}
	else{
		autostep++;
	}
}

driveLeftMasterPercentVoltage = srxDtLm->Get();
driveLeftSlaveOnePercentVoltage = srxDtLs1->Get();
driveLeftSlaveTwoPercentVoltage = srxDtLs2->Get();
driveRightMasterPercentVoltage = srxDtRm->Get();
driveRightSlaveOnePercentVoltage = srxDtRs1->Get();
driveRightSlaveTwoPercentVoltage = srxDtRs2->Get();

frc::SmartDashboard::PutNumber("driveLeftMasterPercentVoltage",driveLeftMasterPercentVoltage);
frc::SmartDashboard::PutNumber("driveLeftSlaveOnePercentVoltage",driveLeftSlaveOnePercentVoltage);
frc::SmartDashboard::PutNumber("driveLeftSlaveTwoPercentVoltage",driveLeftSlaveTwoPercentVoltage);
frc::SmartDashboard::PutNumber("driveRightMasterPercentVoltage",driveRightMasterPercentVoltage);
frc::SmartDashboard::PutNumber("driveRightSlaveOnePercentVoltage",driveRightSlaveOnePercentVoltage);
frc::SmartDashboard::PutNumber("driveRightSlaveTwoPercentVoltage",driveRightSlaveTwoPercentVoltage);
}

void DriveManager::TurnWatch(int angle, double waitTime){  //turns during the auto and boosts turning if it gets stuck
	double z;
	double turnk = -0.012; //Bigger Numbers ARE FASTER (away from zero) //was 17
	double want = angle;
	double time = 0;
	//double allowederror = 5;
	if (loopCount == 0) {  //starts the timer and delays getting the value to avoid errors
	turnTimer->Start();
	}
	if (loopCount > 0) {
	time = turnTimer->Get();
	}

	if (time >= waitTime) {
		boost = boost - 0.0005; //controls how fast the boost ramps up (bigger is faster)
	}
	else {
		boost = 0;
	}

	gyro = ahrs->GetAngle();
	z = ((gyro - want) * (turnk + boost)); //adding the boost
	loopCount++;
	if(fabs(gyro-angle)< 4){  //hardcode 5 as tolerance
		turnTimer->Stop();
		turnTimer->Reset();
		loopCount = 0;
		boost = 0;
		autostep++;
	}

	m_robotDrive->ArcadeDrive(0, z);
	frc::SmartDashboard::PutNumber("Gyro", gyro);
	frc::SmartDashboard::PutNumber("Want", want);
	frc::SmartDashboard::PutNumber("autoTurnLoopCount", loopCount);
	frc::SmartDashboard::PutNumber("autoTurnBoost", boost);
	frc::SmartDashboard::PutNumber("autoTurnTimer", time);
	frc::SmartDashboard::PutNumber("autoTurnWaitTime", waitTime);
}

//a delay for the auto or skips the step it it takes to long, it informs the driver station if it had to skip a step
void DriveManager::autoDelay(int delay, bool skipOrWait) {
	double delayTime = 0;
	if (delayLoop == 0) {
	delayTimer->Start();
	}
	if (delayLoop > 0){
	delayTime = delayTimer->Get();
	}

	delayLoop++;
	if (delay <= delayTime) { //determines if the time meets the required delay
		delayTimer->Stop();
		delayTimer->Reset();
		delayLoop = 0;
		autostep++;
		if (skipOrWait) {
			skipCount++;
		}
	}
	if (skipCount >= 3) { // if it skips to many steps in auto stop the auto
		autostep = 9001; //IT'S OVER 9000!!!!!!!!!!!!!!
	}
}

