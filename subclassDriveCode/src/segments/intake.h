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


	Joystick *stick;

public:
	IntakeManager();
	void Intake();
};

#endif /* SRC_SUBCLASS_INTAKE_H_ */
