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
int timer = 0;
bool dir = false; //false = left, true = right

PIDVariables lateralPID;
PIDVariables rotatePID;

int drivePID(){
  while(autoEngaged){
    if(!movementFinished){
      timer++;
      double leftSideVelocity = 0;
      double rightSideVelocity = 0;
      double leftSideAvg = -(LeftMotor1.position(vex::degrees) + LeftMotor2.position(vex::degrees)) / 2;
      double rightSideAvg = (RightMotor1.position(vex::degrees) + RightMotor2.position(vex::degrees)) / 2;
      double currentPos = (leftSideAvg + rightSideAvg)  / 2;
      double lateralError = lateralPID.target - currentPos;

      double currentHeading = Inertial.heading(); //find current heading
      if(dir){
        currentHeading = currentHeading <= 90 ? currentHeading + 360 : currentHeading; 
        //if turning left, and the current heading is between 0 and 90, then convert the heading to the next positive coterminal angle. 
        //this allows us to calculate our error correctly.
      }else{
        currentHeading = currentHeading >= 270 ? currentHeading - 360 : currentHeading;
      }
      double rotationalError = rotatePID.target - currentHeading;

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

        lateralPID.lastError = lateralPID.error;
      }else if(rotatePID.target != 0 && fabs(rotationalError) > 3){

        confirmSecs = 0;
        rotatePID.error = rotationalError;
        rotatePID.derivative = rotatePID.error - rotatePID.lastError;
        rotatePID.integral += rotatePID.error;

        double velocity = rotatePID.error * rotatePID.kP + rotatePID.integral * rotatePID.kI + rotatePID.derivative * rotatePID.kD;


        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1,1);
        Controller.Screen.print(velocity);
        Controller.Screen.newLine();
        Controller.Screen.print(rotatePID.target);
        Controller.Screen.newLine();
        Controller.Screen.print(rotatePID.error);


        LeftMotors.spin(vex::forward, velocity, vex::percent);
        RightMotors.spin(vex::forward, velocity, vex::percent);

        rotatePID.lastError = rotatePID.error;
      }else{
        confirmSecs++;
        if(confirmSecs == 3){
          Controller.Screen.clearScreen();
          Controller.Screen.setCursor(0, 0);
          Controller.Screen.print("finished");
          Controller.Screen.newLine();
          Controller.Screen.print(timer/10);

          confirmSecs = 0;
          movementFinished = true;
          lateralPID.target = 0;
          rotatePID.target = 0;
          lateralPID.lastError = 0;
          rotatePID.lastError = 0;


          LeftMotors.stop();
          RightMotors.stop();
          timer = 0;
        }
      }
    }
    vex::task::sleep(100);
  }
  return 1;
}

void driveTo(double target){
  lateralPID.target = -(target/WHEEL_CIRCUMFRENECE)*360;
  movementFinished = false;
  
  LeftMotor1.resetPosition();
  LeftMotor2.resetPosition();
  RightMotor1.resetPosition();
  RightMotor2.resetPosition();
}
void rotateTo(double target){ 
  rotatePID.target = target < 0 ? 360 + target : target; //if we are turning left, convert the target from negative degrees to its principal angle.
  movementFinished = false;
  dir = target < 0;

  Inertial.resetHeading();
}


void preAuton(){
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Calibrating inertial");
  Inertial.calibrate();
  while(Inertial.isCalibrating()){
    vex::task::sleep(100);
  }
  Inertial.resetHeading();
  LeftMotor1.resetPosition();
  LeftMotor2.resetPosition();
  RightMotor1.resetPosition();
  RightMotor2.resetPosition();
  Controller.Screen.newLine();
  Controller.Screen.print("Finished calibrating");
  Controller.Screen.newLine();
  autoEngaged = false;

  lateralPID.target = 0;
  lateralPID.kP = 0.1;
  lateralPID.kI = 0;
  lateralPID.kD = 0.0;
  lateralPID.lastError = 0;

  rotatePID.target = 0;
  rotatePID.kP = 0.372;
  rotatePID.kI = 0.0052;
  rotatePID.kD = 0.2;
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
      Controller.Screen.print("Skills Routine");
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
  if(routine == 1){//far side
    driveTo(50);
    Lift.setVelocity(-100, vex::percent);
    Lift.spinFor(100, vex::degrees, false);
    waitUntil(movementFinished);
    rotateTo(75);
    waitUntil(movementFinished);
    Intake.spin(vex::reverse, 100, vex::percent);
    vex::task::sleep(50);
    Intake.stop();
    driveTo(-10);
    waitUntil(movementFinished);
    rotateTo(-85);
    waitUntil(movementFinished);
    Intake.spin(vex::forward, 100, vex::percent);
    driveTo(10);
    waitUntil(Distance.objectDistance(vex::mm) < 170);
    Intake.stop();
    rotateTo(95);
    waitUntil(movementFinished);
    LeftMotors.spin(vex::forward, 100, vex::percent);
    RightMotors.spin(vex::reverse, 100, vex::percent);
    Intake.spin(vex::reverse, 100, vex::percent);
    vex::task::sleep(760);
    LeftMotors.stop();
    RightMotors.stop();
    Intake.stop();
    driveTo(-5);
  }else if(routine == 2){

  }else if(routine == 3){ //skills
    Lift.setVelocity(-100, vex::percent);
    Lift.spinFor(200, vex::degrees, false);
    Flywheel.spin(vex::reverse, 100, vex::percent);
    int gameTime = 0;
    while(gameTime != 50){ //CHANGE TO 55 SECONDS
      gameTime++;
      vex::task::sleep(1000);
    }
    Flywheel.stop();
    driveTo(-5);
    waitUntil(movementFinished);
    rotateTo(-90);
    waitUntil(movementFinished);
    driveTo(-25);
    waitUntil(movementFinished);
    rotateTo(45);
    waitUntil(movementFinished);
    LeftMotors.spin(vex::reverse, 100, vex::percent);
    RightMotors.spin(vex::forward, 100, vex::percent);
    vex::task::sleep(750);
    LeftMotors.stop();
    RightMotors.stop();
    driveTo(5);
    waitUntil(movementFinished);
    LeftMotors.spin(vex::reverse, 100, vex::percent);
    RightMotors.spin(vex::forward, 100, vex::percent);
    vex::task::sleep(750);
    LeftMotors.stop();
    RightMotors.stop();
  }
}