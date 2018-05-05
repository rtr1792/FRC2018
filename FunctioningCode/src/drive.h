/*
 * drive.h
 *
 *  Created on: Jan 24, 2018
 *      Author: RTR
 */


#ifndef SRC_SUBCLASS_DRIVE_H_
#define SRC_SUBCLASS_DRIVE_H_
#include "AHRS.h"

class DriveManager {
private:
		WPI_TalonSRX *srxDtLm;
		WPI_TalonSRX *srxDtLs1;
		WPI_TalonSRX *srxDtLs2;

		WPI_TalonSRX *srxDtRm;
		WPI_TalonSRX *srxDtRs1;
		WPI_TalonSRX *srxDtRs2;

		DifferentialDrive *m_robotDrive;

		Joystick *stick;
		XboxController *xbox;

		Timer *turnTimer;
		Timer *delayTimer;

		double *rightStickValue;
		double *leftStickValue;
		double *vel1;
		double *vel2;
		double *dist;
		double *dist2;
		int *init;
		int *one;

	    AHRS *ahrs;
public:
	DriveManager();
	void driveTrain();
	void Drive(double speed, double goDistance);
	void Turn(int angle);
	void ResetSensors();
	void setCoast();
	void FindStartEnc();
	void DriveNew(double speed, double goDistance);
	void TurnWatch(int angle, double waitTime); //enhanced turn to increase power after a certain amount of time
	void autoDelay(int delay, bool skipOrWait);
};
//	null ArcadeDrive(double, double, bool);

#endif /* SRC_SUBCLASS_DRIVE_H_ */
