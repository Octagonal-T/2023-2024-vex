#include <vex.h>
#ifndef CONSTANTS_H_
#define CONSTANTS_H_

extern vex::brain Brain;
extern vex::competition Competition;
extern vex::controller Controller;

extern vex::motor LeftMotor1;
extern vex::motor LeftMotor2;
extern vex::motor RightMotor1;
extern vex::motor RightMotor2;

extern vex::motor Intake;
extern vex::motor Flywheel;
extern vex::motor Lift;
extern vex::motor Wings;

extern vex::inertial Inertial;
extern vex::distance Distance;

struct PIDVariables{
  double kP;
  double kI;
  double kD;

  double error;
  double lastError;
  double derivative;
  double integral;

  double target;
  double tolerance;
};

extern vex::motor_group LeftMotors;
extern vex::motor_group RightMotors;
#endif