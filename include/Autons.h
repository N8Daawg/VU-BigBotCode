/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Drive_PDs.h                                               */
/*    Author:       Nathan Beals                                              */
/*    Created:      Thu Feb. 23 2023                                          */
/*    Description:  Autonamous codes                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "voids.h"
#include "Drive_PDS.h"
#include "VisionVoids.h"

/**
 * @brief simple move forward while other voids are being fixed
 * 
 * @param pos target to turn to in motor degrees
*/
void RunDriveTrain(int pos){
  FL.spinFor(pos, degrees, false);FR.spinFor(pos, degrees, false);
  ML.spinFor(pos, degrees, false);MR.spinFor(pos, degrees, false);
  BL.spinFor(pos, degrees, false);BR.spinFor(pos, degrees, true);
  resetDrivePositions();
}

/**
 * @brief simple move forward while other voids are being fixed
 * 
 * @param vel velocity of the robot
*/
void RunFwd(int vel){
  FL.spin(fwd,vel,pct);FR.spin(fwd,vel*0.5,pct);
  ML.spin(fwd,vel,pct);MR.spin(fwd,vel*0.5,pct);
  BL.spin(fwd,vel,pct);BR.spin(fwd,vel*0.5,pct);
  resetDrivePositions();
}

/**
 * @brief simple reverse while other voids are being fixed
 * 
 * @param vel velocity of the robot
*/
void RunRev(int vel){
  FL.spin(reverse,vel,pct);FR.spin(reverse,vel,pct);
  ML.spin(reverse,vel,pct);MR.spin(reverse,vel,pct);
  BL.spin(reverse,vel,pct);BR.spin(reverse,vel,pct);
  resetDrivePositions();
}
/**
 * @brief simple point turn while other voids are being fixed
 * 
 * @param pos position to turn to
 * @param dir direction to turn
*/
void TurnDriveTrain(int pos, bool dir){
  if (dir) pos = -pos;
  FL.spinFor(pos, degrees, false);FR.spinFor(-pos, degrees, false);
  ML.spinFor(pos, degrees, false);MR.spinFor(-pos, degrees, false);
  BL.spinFor(pos, degrees, false);BR.spinFor(-pos, degrees, true);
  resetDrivePositions();
}

/***
 * @brief a void for autonomous test cases
*/
void test(){
  //DrivePD(50);
  arcTurnRight(robotWidth/2, 30, 50);
  //reload();
  //before and after distance should be 25sqrt(2) inches
  
}

/**
 * @brief a void containing match autonomous code
*/
void match(){
  RunDriveTrain(1200);
  Lwing.open();
  pointTurnLeft(-90, 50);
  RunFwd(50);
  wait(1, sec);
  StopDriveTrain(brake);
  Lwing.close();
  RunRev(50);
  wait(1,sec);
  RwingToggle();
  pointTurnLeft(60, 50);
  moverev(15, 50);
}

/**
 * @brief a void containing skills autonomous code
*/
void skills(){
  for (int i = 0; i<10; i++){
    Rwing.open();
    wait(0.5,sec);
    Rwing.close();
    wait(1.0,sec);
  }
  resetDrivePositions();
  LWingToggle();
  arcTurnRight(robotWidth/2, 60, 50);
  wait(1,sec);
  //Brain.Screen.print(Gyro.rotation());
  //GyroTurn_PD(20, false); // we need to fix this function
  SetDriveTrainVelocity(100);
  //Brain.Screen.print(Gyro.rotation());
  RunDriveTrain(1400);
  RwingToggle();
  arcTurnRight(robotWidth/2, 45, 50);
  RunFwd(100);
  wait(3,sec);
  StopDriveTrain(brake);
  wait(0.5,sec);
  RunDriveTrain(-300);
}