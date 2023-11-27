#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>

int intakeDirection = -1; //-1 = not spinning, 0 = forward, 1 = reverse
bool flywheelOn = false;
bool wingsOn = false;

void intake(){
  if(intakeDirection != 0){
    Intake.spin(vex::forward, 100, vex::percent);
    intakeDirection = 0;
  }else{
    Intake.stop();
    intakeDirection = -1;
  }
}
void reverseIntake(){
  if(intakeDirection != 1){
    Intake.spin(vex::reverse, 100, vex::percent);
    intakeDirection = 1;
  }else{
    Intake.stop();
    intakeDirection = -1;
  }
}
void toggleFlywheel(){
  if(flywheelOn){
    Flywheel.stop();
  }else{
    Flywheel.spin(vex::forward, 100, vex::percent);
  }
  flywheelOn = !flywheelOn;
}
void toggleWings(){
  if(wingsOn){
    
  }
}