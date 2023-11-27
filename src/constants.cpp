#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>

vex::brain Brain;
vex::competition Competition;
vex::controller Controller(vex::primary);

vex::motor LeftMotor1(vex::PORT18, vex::ratio18_1, false); //front
vex::motor LeftMotor2(vex::PORT17, vex::ratio18_1, true); //back
vex::motor RightMotor1(vex::PORT20, vex::ratio18_1, true); //front
vex::motor RightMotor2(vex::PORT19, vex::ratio18_1, false); //back

vex::motor Intake(vex::PORT16, vex::ratio18_1, false);
vex::motor Flywheel(vex::PORT15, vex::ratio6_1, true);
vex::motor Lift(vex::PORT14, vex::ratio36_1, false);

vex::inertial Inertial(vex::PORT10);

vex::motor_group LeftMotors(LeftMotor1, LeftMotor2);
vex::motor_group RightMotors(RightMotor1, RightMotor2);