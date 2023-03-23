#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vex.h>
#include <constants.h>

PIDVariables flywheelPID;
bool flywheelEngaged = false;
int launcherPID(){
  while(true){
    if(flywheelEngaged){
      double current = Flywheel.velocity(vex::rpm);

      flywheelPID.error = current - flywheelPID.target;
      flywheelPID.integral += flywheelPID.error;
      flywheelPID.derivative = flywheelPID.error - flywheelPID.lastError;

      double flywheelVelocity = flywheelPID.error * flywheelPID.kP + flywheelPID.derivative * flywheelPID.kD + flywheelPID.integral * flywheelPID.kI;
      if(flywheelVelocity > 12) flywheelVelocity = 12;

      Flywheel.spin(vex::forward, flywheelVelocity, vex::volt);
      flywheelPID.lastError = flywheelPID.error;
    }
    vex::task::sleep(20);
  }
  return 1;
}
void toggleFlywheel(int rpm){
  flywheelPID.kP = 0.1;
  flywheelPID.kI = 0.05;
  flywheelPID.kD = 0.05;

  if(flywheelEngaged){
    flywheelPID.target = 0;
  }else{
    flywheelPID.target = rpm;
  }
  flywheelEngaged = !flywheelEngaged;
}
void toggleIndexer(){
  Indexer.setVelocity(50, vex::percent);
  Indexer.spinFor(-0.2, vex::rev, true);
  Indexer.spinFor(0.2, vex::rev, true);  
}