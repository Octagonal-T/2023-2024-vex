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

vex::motor LeftMotor1(vex::PORT1, vex::ratio18_1, true);
vex::motor LeftMotor2(vex::PORT2, vex::ratio18_1, true);
vex::motor RightMotor1(vex::PORT3, vex::ratio18_1, false);
vex::motor RightMotor2(vex::PORT4, vex::ratio18_1, false);
vex::motor Flywheel(vex::PORT5, vex::ratio6_1, true);
vex::motor Indexer(vex::PORT6, vex::ratio18_1, false);
vex::inertial Inertial(vex::PORT7);
vex::encoder Encoder(Brain.ThreeWirePort.G);

vex::motor_group LeftMotors(LeftMotor1, LeftMotor2);
vex::motor_group RightMotors(RightMotor1, RightMotor2);