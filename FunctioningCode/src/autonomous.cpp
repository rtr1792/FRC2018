/*
 * autonomous.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Scouting1792-PC
 */
#include <WPILib.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <Joystick.h>
#include <Talon.h>
#include "autonomous.h"
#include "math.h"
#include <Timer.h>
#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SpeedController.h>
#include <Drive/DifferentialDrive.h>
#include <DriverStation.h>
#include <Encoder.h>
#include "AHRS.h"
#include <SPI.h>

//Crosses the baseline, nothing more
AutoManager::AutoManager() {

}

void AutoManager::Auto1() {

}

//starts on left side, delivers cube to switch when on the same side
void AutoManager::Auto2() {
}

//start in rightmost position, deliver cube to same side
void AutoManager::Auto3() {
}

//start in middle, deliver to right side of switch
void AutoManager::Auto4() {
}



