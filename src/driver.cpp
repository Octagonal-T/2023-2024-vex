#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>
#include <subtasks.h>


bool leftMotorMoving = true;
bool rightMotorMoving = true;
bool driverEnabled = false;
bool liftMoving = false;
bool wingsMoving = false;
int gameTime = 0;
int delayIntakeStop = 0;

int updateControllerScreen(){
  while(driverEnabled){
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("Intake1: %dV, %dW", floor(Intake.voltage(vex::volt)), floor(Intake.power(vex::watt)));
    Controller.Screen.newLine();
    Controller.Screen.print("Flywheel: %dV, %dW", floor(Flywheel.voltage(vex::volt)), floor(Flywheel.power(vex::watt)));
    Controller.Screen.newLine();
    int seconds = floor(gameTime / 50);
    int mins = floor(seconds / 60);
    if(seconds == 95) Controller.rumble("..");
    seconds = seconds - (mins*60);
    Controller.Screen.newLine();
    Controller.Screen.print("Game time: %d:%d", mins, seconds);
    vex::task::sleep(250);
  }
  return 1;
}

void driverControl(){
  driverEnabled = true;
  // Controller.ButtonL2.pressed(toggleWings);
  Controller.ButtonX.pressed(toggleFlywheel);
  Controller.ButtonR1.pressed(intake);
  vex::task controllerScreen(updateControllerScreen);
  while(true){
    Lift.setBrake(vex::hold);
    if(Distance.objectDistance(vex::mm) < 170 && intakeDirection == 1){
      if(delayIntakeStop == 3){
        Intake.stop();
        intakeDirection = 0;
        delayIntakeStop = 0;
      }else{
        delayIntakeStop++;
      }
    }
    
    //lift
    if(Controller.ButtonL2.pressing()){
      if(Controller.ButtonL1.pressing()){
        Lift.spin(vex::forward, 100, vex::percent);
      }else{
        Lift.spin(vex::reverse, 100, vex::percent);
      }
      liftMoving = true;
    }else if(liftMoving){
      liftMoving = false;
      Lift.stop();
    }
    //wings
    if(Controller.ButtonR2.pressing()){
      if(Controller.ButtonL1.pressing()){
        Wings.spin(vex::forward, 100, vex::percent);
      }else{
        Wings.spin(vex::reverse, 100, vex::percent);
      }
      wingsMoving = true;
    }else if(wingsMoving){
      wingsMoving = false;
      Wings.stop();
    }
    double leftJoystick = abs(Controller.Axis1.position()) > 5 ? Controller.Axis1.position() : 0;
    double rightJoystick = abs(Controller.Axis3.position()) > 5 ? Controller.Axis3.position() : 0;

    double leftSideSpeed = leftJoystick + rightJoystick;
    double rightSideSpeed = leftJoystick - rightJoystick;

    if(abs(leftJoystick) > 90 && abs(rightJoystick) > 90){
      leftSideSpeed = (abs(leftSideSpeed) < 5) ? ((leftSideSpeed > 0 && leftSideSpeed < 5) ? -25: 25) : leftSideSpeed;
      rightSideSpeed = (abs(rightSideSpeed) < 5) ? ((rightSideSpeed > 0 && rightSideSpeed < 5) ? -25: 25) : rightSideSpeed;
    }
    if (fabs(leftSideSpeed) < 5) {
      if (leftMotorMoving) {
        LeftMotors.stop();
        leftMotorMoving = false;
      }
    } else {
      leftMotorMoving = true;
    }
    if (fabs(rightSideSpeed) < 5) {
      if (rightMotorMoving) {
        RightMotors.stop();
        rightMotorMoving = false;
      }
    } else {
      rightMotorMoving = true;
    }
    if (leftMotorMoving) {
      LeftMotors.spin(vex::forward, leftSideSpeed, vex::percent);
    }
    if (rightMotorMoving) {
      RightMotors.spin(vex::forward, rightSideSpeed, vex::percent);
    }

    gameTime++;
    vex::task::sleep(20);
  }
}