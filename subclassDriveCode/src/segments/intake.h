/*
 * intake.h
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

#ifndef SRC_SUBCLASS_INTAKE_H_
#define SRC_SUBCLASS_INTAKE_H_

class IntakeManager {
private:
	WPI_TalonSRX *srx1;
	WPI_TalonSRX *srx2;
//	Ultrasonic *ult;
//	Ultrasonic *ult2;

	Joystick *stick;
	XboxController *xbox;

/*	int *rd;
	int *ld;
	int *zero;
	int *one; */
	int *reverse;
public:
	IntakeManager();
	void Intake();
};

#endif /* SRC_SUBCLASS_INTAKE_H_ */
