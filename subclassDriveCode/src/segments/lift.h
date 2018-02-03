/*
 * lift.h
 *
 *  Created on: Jan 27, 2018
 *      Author: RTR
 */

#ifndef SRC_SUBCLASS_LIFT_H_
#define SRC_SUBCLASS_LIFT_H_

class LiftManager {
private:
	WPI_TalonSRX *srx1;

	Joystick *stick;
	XboxController *xbox;

	DigitalInput *limit;
	DigitalInput *limit2;

	int *liftValue;
	bool *button3;
	bool *button4;
	double *encoder;
	int *zero;
	int *one;
	int *two;
	int *three;
	int *four;
public:
	LiftManager();
	void Lift();
};

#endif /* SRC_SUBCLASS_LIFT_H_ */
