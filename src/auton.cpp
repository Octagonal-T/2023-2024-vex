#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>
#include <subtasks.h>
#include <auton.h>

routines routine;
PIDVariables lateralPID;
PIDVariables rotationPID;
bool movementFinished;
int confirmSeconds;

void driveTo(double lateral, double heading){
  lateralPID.target = lateral/8.6393798;
  rotationPID.target = heading;
  movementFinished = false;
}
int drivePID(){
  while(true){
    double pos = Encoder.rotation(vex::rev) * -1;
    // double currentHeading = Inertial.heading(vex::degrees);

    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);

    lateralPID.error = pos - lateralPID.target;
    // rotationPID.error = currentHeading - rotationPID.target;

    Controller.Screen.print(pos);
    Controller.Screen.newLine();
    Controller.Screen.print(lateralPID.error);
    Controller.Screen.newLine();
    
    if(!(lateralPID.error < 0.01 && lateralPID.error > -0.01) || !(rotationPID.error < 0.01 && rotationPID.error > -0.01)){
      confirmSeconds=0;
      movementFinished = false;
      lateralPID.derivative = lateralPID.error - lateralPID.lastError;
      rotationPID.derivative = rotationPID.error - rotationPID.lastError;
      lateralPID.integral+=lateralPID.error;
      rotationPID.integral += rotationPID.error;
      double lateralVelocity = (lateralPID.error * lateralPID.kP + lateralPID.derivative * lateralPID.kD + lateralPID.integral * lateralPID.kI);
      // double turnVelocity = (rotationPID.error * rotationPID.kP + rotationPID.derivative * rotationPID.kD + rotationPID.integral * rotationPID.kI);
      
      double leftMotorsVelocity = (lateralVelocity );
      double rightMotorsVelocity = (lateralVelocity );
      Controller.Screen.print(lateralVelocity);
      if(leftMotorsVelocity > 12) leftMotorsVelocity = 12;
      else if(leftMotorsVelocity < -12) leftMotorsVelocity = -12;
      if(rightMotorsVelocity > 12) rightMotorsVelocity = 12;
      else if(rightMotorsVelocity < -12) rightMotorsVelocity = -12;

      LeftMotors.spin(vex::forward, leftMotorsVelocity, vex::volt);
      RightMotors.spin(vex::forward, rightMotorsVelocity, vex::volt);

      lateralPID.lastError = lateralPID.error;
      rotationPID.lastError = rotationPID.error;
    }else{
      confirmSeconds++;
      if(confirmSeconds==5){
        confirmSeconds = 0;
        movementFinished = true;
        Inertial.resetHeading();
        Encoder.resetRotation();
        lateralPID.target = 0;
        rotationPID.target = 0;
        lateralPID.lastError = 0;
        rotationPID.lastError = 0;
        LeftMotors.stop();
        RightMotors.stop();
      }
    }
    vex::task::sleep(20);
  }
  return 1;
}

void preAuton(){
  lateralPID.kP = 2;
  lateralPID.kI = 0.0;
  lateralPID.kD = 0.0;

  rotationPID.kP = 0.1;
  rotationPID.kI = 0.05;
  rotationPID.kD = 0.05;

  movementFinished = true;
  confirmSeconds = 0;

}
void startAutonomous(){
  vex::task drivePIDTask(drivePID);
  // vex::task flywheelPID(launcherPID);
  driveTo(100, 0);
  // toggleFlywheel(600);
  // driveTo(100);
  // waitUntil(movementFinished);
  toggleIndexer();
}