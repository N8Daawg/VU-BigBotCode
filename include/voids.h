/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Voids.h                                                   */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  Autonimous voids                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
extern double ratio;
extern double robotWidth;

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
void coastDriveTrain(){
  FL.stop(brakeType::coast);FR.stop(brakeType::coast);
  ML.stop(brakeType::coast);MR.stop(brakeType::coast);
  BL.stop(brakeType::coast);BR.stop(brakeType::coast);
}

/*---------------------------------------------------------------------------*/
/*----------------------------Drivetrain Movements---------------------------*/
/*---------------------------------------------------------------------------*/

void movefwd(int  goal, int Velocity ){
  SetDriveTrainVelocity(Velocity);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(forward, theta, degrees, false);FR.spinFor(forward, theta, degrees,  false);
  ML.spinFor(forward, theta, degrees, false);MR.spinFor(forward, theta, degrees,  false);
  BL.spinFor(forward, theta, degrees, false);BR.spinFor(forward, theta, degrees,  true);
  
  StopDriveTrain();
}


void moverev( int goal, int velocityunit ){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(reverse, theta, degrees, false);FR.spinFor(reverse, theta, degrees,  false);
  ML.spinFor(reverse, theta, degrees, false);MR.spinFor(reverse, theta, degrees,  false);
  BL.spinFor(reverse, theta, degrees, false);BR.spinFor(reverse, theta, degrees,  true);
  
  StopDriveTrain();
}

void arcTurnLeft(double radius, double theta ,int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double leftDistance = theta*(radius-(robotWidth/2));
  double rightDistance = theta*(radius+(robotWidth/2));

  leftDistance = leftDistance/ratio;rightDistance=rightDistance/ratio;

  FL.spinFor(forward, leftDistance, degrees, false);FR.spinFor(forward, rightDistance, degrees,  false);
  ML.spinFor(forward, leftDistance, degrees, false);MR.spinFor(forward, rightDistance, degrees,  false);
  BL.spinFor(forward, leftDistance, degrees, false);BR.spinFor(forward, rightDistance, degrees,  true);
  
  

  holdDriveTrain();
}

void arcTurnRight(double radius, double theta ,int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double leftDistance = theta*(radius+(robotWidth/2));
  double rightDistance = theta*(radius-(robotWidth/2));

  leftDistance = leftDistance/ratio;rightDistance=rightDistance/ratio;

  FL.spinFor(forward, leftDistance, degrees, false);FR.spinFor(forward, rightDistance, degrees,  false);
  ML.spinFor(forward, leftDistance, degrees, false);MR.spinFor(forward, rightDistance, degrees,  false);
  BL.spinFor(forward, leftDistance, degrees, false);BR.spinFor(forward, rightDistance, degrees,  true);

  holdDriveTrain();
}

void pointTurnLeft(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();
  double goal = theta*(robotWidth/2);
  goal = goal/ratio;

  FL.spinFor(reverse, goal, degrees, false);FR.spinFor(forward, goal, degrees,  false);
  ML.spinFor(reverse, goal, degrees, false);MR.spinFor(forward, goal, degrees,  false);
  BL.spinFor(reverse, goal, degrees, false);BR.spinFor(forward, goal, degrees,  true);
  
  holdDriveTrain();
}

void pointTurnRight(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();
  double goal = theta*(robotWidth/2);
  goal = goal/ratio;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  ML.spinFor(forward, goal, degrees, false);MR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  
  holdDriveTrain();
}

void turnleft(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  FL.spinFor(reverse, theta, degrees, false);FR.spinFor(forward, theta, degrees,  false);
  ML.spinFor(reverse, theta, degrees, false);MR.spinFor(forward, theta, degrees,  false);
  BL.spinFor(reverse, theta, degrees, false);BR.spinFor(forward, theta, degrees,  true);
  
  StopDriveTrain();
}


void turnright(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  FL.spinFor(forward, theta, degrees, false);FR.spinFor(reverse, theta, degrees,  false);
  ML.spinFor(forward, theta, degrees, false);MR.spinFor(reverse, theta, degrees,  false);
  BL.spinFor(forward, theta, degrees, false);BR.spinFor(reverse, theta, degrees,  true);
  
  holdDriveTrain();
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