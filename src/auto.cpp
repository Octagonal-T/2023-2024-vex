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
double WHEEL_CIRCUMFRENECE = 12.5663706;
int routine;
bool autoEngaged;
bool movementFinished = true;
int confirmSecs = 0;

PIDVariables lateralPID;
PIDVariables rotatePID;

int drivePID(){
  while(autoEngaged){
    if(!movementFinished){
      double leftSideVelocity = 0;
      double rightSideVelocity = 0;
      double leftSideAvg = -(LeftMotor1.position(vex::degrees) + LeftMotor2.position(vex::degrees)) / 2;
      double rightSideAvg = (RightMotor1.position(vex::degrees) + RightMotor2.position(vex::degrees)) / 2;
      double currentPos = (leftSideAvg + rightSideAvg)  / 2;
      double lateralError = lateralPID.target - currentPos;
      if(lateralPID.target != 0 && fabs(lateralError) > 25){
        confirmSecs = 0;
        lateralPID.error = lateralPID.target - currentPos;
        lateralPID.derivative = lateralPID.error - lateralPID.lastError;
        lateralPID.integral += lateralPID.error;
        leftSideVelocity = -(lateralPID.error * lateralPID.kP + lateralPID.derivative * lateralPID.kD + lateralPID.integral * lateralPID.kI);
        rightSideVelocity = (lateralPID.error * lateralPID.kP + lateralPID.derivative * lateralPID.kD + lateralPID.integral * lateralPID.kI);
        
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1,1);
        Controller.Screen.print(leftSideVelocity);
        Controller.Screen.newLine();
        Controller.Screen.print(lateralPID.target);
        Controller.Screen.newLine();
        Controller.Screen.print(lateralPID.error);
        LeftMotors.spin(vex::forward, leftSideVelocity, vex::percent);
        RightMotors.spin(vex::forward, rightSideVelocity, vex::percent);
      }else if(rotatePID.target != 0){

      }else{
        confirmSecs++;
        if(confirmSecs == 15){
          Controller.Screen.clearScreen();
          Controller.Screen.print("finished");
          confirmSecs = 0;
          movementFinished = true;
          lateralPID.target = 0;
          rotatePID.target = 0;
          lateralPID.lastError = 0;
          rotatePID.lastError = 0;
          LeftMotors.stop();
          RightMotors.stop();
        }
      }
    }
  }
  return 1;
}

void driveTo(double target){
  lateralPID.target = -(target/WHEEL_CIRCUMFRENECE)*360;
  movementFinished = false;
}

void preAuton(){
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Calibrating inertial");
  // Inertial.calibrate();
  // while(Inertial.isCalibrating()){
  //   vex::task::sleep(100);
  // }
  // Encoder.resetRotation();
  // Inertial.resetHeading();
  LeftMotor1.resetPosition();
  LeftMotor2.resetPosition();
  RightMotor1.resetPosition();
  RightMotor2.resetPosition();
  Controller.Screen.newLine();
  Controller.Screen.print("Finished calibrating");
  Controller.Screen.newLine();
  autoEngaged = false;

  lateralPID.kP = 0.1;
  lateralPID.kI = 0;
  lateralPID.kD = 0.0;
  lateralPID.lastError = 0;

  rotatePID.kP = 0.4;
  rotatePID.kI = 0;
  rotatePID.kD = 0.0;
  rotatePID.lastError = 0;

  Controller.Screen.print("Set PID variables");
  vex::task::sleep(1000);

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Left: far routine");
  Controller.Screen.newLine();
  Controller.Screen.print("Right: Close routine");
  Controller.Screen.newLine();
  Controller.Screen.print("X: no routine");
  bool flag = false;
  for (int i = 0; i < 500; i++){
    if(Controller.ButtonLeft.pressing()){
      routine = 1;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Far selected");
      flag = true;
      break;
    }else if(Controller.ButtonRight.pressing()){
      routine = 2;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Close selected");
      flag = true;
      break;
    }else if(Controller.ButtonX.pressing()){
      routine = 0;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("No routine");
      flag = true;
      break;
    }else if(Controller.ButtonDown.pressing()){
      routine = 3;
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Test routine");
      flag = true;
      break;

    }
    vex::task::sleep(10);
  }
  if(!flag){
    routine = 0;
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("No routine");
  }
  vex::task::sleep(1000);
  Controller.Screen.clearScreen();
  Brain.Screen.drawImageFromFile("dougal.png", 0, 0);
}

void startAutonomous(){
  autoEngaged = true;
  vex::task drivePIDTask(drivePID);
  driveTo(39);
}