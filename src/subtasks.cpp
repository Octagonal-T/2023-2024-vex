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
  if(Controller.ButtonL1.pressing()){
    if(intakeDirection != -1 || intakeDirection != 1){
      Intake.spin(vex::reverse, 100, vex::percent);
      intakeDirection = -1;
    }else{
      Intake.stop();
      intakeDirection = 0;
    }
  }else{
    if(intakeDirection != 1){
      Intake.spin(vex::forward, 100, vex::percent);
      intakeDirection = 1;
    }else{
      Intake.stop();
      intakeDirection = 0;
    }
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