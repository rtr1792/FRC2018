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
public:
	LiftManager();
	void Lift();
};

#endif /* SRC_SUBCLASS_LIFT_H_ */
