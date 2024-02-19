/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Voids.h                                                   */
/*    Author:       Nathan Beals                                              */
/*    Created:      sat Feb. 18 2023                                          */
/*    Description:  Autonimous voids                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

/*---------------------------------------------------------------------------*/
/*-----------------------Drivetrain Utility Functions------------------------*/
/*---------------------------------------------------------------------------*/

double getPositionAverages(motor front, motor middle, motor back){
  return (front.position(degrees)+middle.position(degrees)+back.position(degrees))/3;
}
 double getVelocityAverages(motor front, motor middle, motor back){
  return (front.velocity(pct)+middle.velocity(pct)+back.velocity(pct))/3;
 }

void SetDriveTrainVelocity(int Velocity) {
  FL.setVelocity(Velocity, pct);FR.setVelocity(Velocity, pct);
  ML.setVelocity(Velocity, pct);MR.setVelocity(Velocity, pct);
  BL.setVelocity(Velocity, pct);BR.setVelocity(Velocity, pct);
}

void resetDrivePositions(){
  FL.resetPosition();FR.resetPosition();
  ML.resetPosition();MR.resetPosition();
  BL.resetPosition();BR.resetPosition();
}


void StopDriveTrain(){
  FL.stop(brakeType::brake);FR.stop(brakeType::brake);
  ML.stop(brakeType::brake);MR.stop(brakeType::brake);
  BL.stop(brakeType::brake);BR.stop(brakeType::brake);
}
void holdDriveTrain(){
  FL.stop(brakeType::hold);FR.stop(brakeType::hold);
  ML.stop(brakeType::hold);MR.stop(brakeType::hold);
  BL.stop(brakeType::hold);BR.stop(brakeType::hold);
}

/*---------------------------------------------------------------------------*/
/*-----------------------------Game Object Utility---------------------------*/
/*---------------------------------------------------------------------------*/

extern int RwingCount;
void RwingToggle(void){
  RwingCount++;
  if (RwingCount%2==0){
    Rwing.close();
  } else{
    Rwing.open();
  }
}

extern int LwingCount;
void LWingToggle(void){
  LwingCount++;
  if (LwingCount%2==0){
    Lwing.close();
  } else{
    Lwing.open();
  }
}