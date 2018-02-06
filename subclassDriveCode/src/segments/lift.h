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
	WPI_TalonSRX *srx2;

	Joystick *stick;
	XboxController *xbox;

	DigitalInput *limit;
	DigitalInput *limit2;

	int *liftValue;
	int *button3;
	int *button4;
	int *limits;
	double *encoder;
	int *zero;
	int *one;
	int *two;
	int *three;
	int *four;
	int *dlimit;
	int *mone;
public:
	LiftManager();
	void Lift();
};

#endif /* SRC_SUBCLASS_LIFT_H_ */
