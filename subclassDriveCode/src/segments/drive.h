/*
 * drive.h
 *
 *  Created on: Jan 24, 2018
 *      Author: RTR
 */


#ifndef SRC_SUBCLASS_DRIVE_H_
#define SRC_SUBCLASS_DRIVE_H_


class DriveManager {
private:
		WPI_TalonSRX *srx1;
		WPI_TalonSRX *srx12;
		WPI_TalonSRX *srx13;

		WPI_TalonSRX *srx2;
		WPI_TalonSRX *srx21;
		WPI_TalonSRX *srx22;

		DifferentialDrive *m_robotDrive;
		DifferentialDrive *m_robotDrive2;
		DifferentialDrive *m_robotDrive3;

		Joystick *stick;

		double *rightStickValue;
		double *leftStickValue;
		double *vel1;
		double *vel2;
public:
	DriveManager();
	void driveTrain();
};
//	null ArcadeDrive(double, double, bool);

#endif /* SRC_SUBCLASS_DRIVE_H_ */
