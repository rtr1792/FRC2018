/*
 * autonomous.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include <Timer.h>
#include <segments/lift.h>

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

class AutoManager {
private:
	WPI_TalonSRX *srx1;  //has enc
	WPI_TalonSRX *srx2;  //left
	WPI_TalonSRX *srx3;

	WPI_TalonSRX *srx4;  //has enc
	WPI_TalonSRX *srx5;  //right
	WPI_TalonSRX *srx6;

	WPI_TalonSRX *intake1; //intake motor (left?)
	WPI_TalonSRX *intake2; //intake motor (right?)

	WPI_TalonSRX *liftSRX1; //lift motor (top?)
	WPI_TalonSRX *liftSRX2; //lift motor (bottom?)

	AHRS *ahrs;

	double pi;
	double constant;

	Timer *timer;
	LiftManager *liftManager;


public:
	AutoManager();
	void Auto1();
	void Auto2();
	void Auto3();
	void Auto4();
};



#endif /* SRC_AUTONOMOUS_H_ */
