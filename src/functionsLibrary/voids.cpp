/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Voids.h                                                   */
/*    Author:       Nathan Beals                                              */
/*    Created:      sat Feb. 18 2023                                          */
/*    Description:  Autonomous voids                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

/*---------------------------------------------------------------------------*/
/*-----------------------Drivetrain Utility Functions------------------------*/
/*---------------------------------------------------------------------------*/

double getHeading(bool dir){
    if(dir){
        return 360 - Gyro.heading();
    } else{
        return Gyro.heading();
    }
}


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


void StopDriveTrain(brakeType Brake){
  FL.stop(Brake);
  BL.stop(Brake);
  FR.stop(Brake);
  BR.stop(Brake);
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
  
  StopDriveTrain(brake);
}


void moverev( int goal, int velocityunit ){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(reverse, theta, degrees, false);FR.spinFor(reverse, theta, degrees,  false);
  ML.spinFor(reverse, theta, degrees, false);MR.spinFor(reverse, theta, degrees,  false);
  BL.spinFor(reverse, theta, degrees, false);BR.spinFor(reverse, theta, degrees,  true);
  
  StopDriveTrain(brake);
}


void arcTurnLeft(int radius, int theta, int velocityunit){
  double goal = (theta*radius)/ratio;

  double leftRadius = radius-(robotWidth/2);
  double rightRadius = radius+(robotWidth/2);
  
  double leftDistance = goal*leftRadius/radius; double leftspeed = velocityunit*(leftRadius/radius);
  double rightDistance = goal*rightRadius/radius; double rightspeed = velocityunit*(rightRadius/radius);
  
  FL.setVelocity(leftspeed, pct);FR.setVelocity(rightspeed, pct);
  ML.setVelocity(leftspeed, pct);MR.setVelocity(rightspeed, pct);
  BL.setVelocity(leftspeed, pct);BR.setVelocity(rightspeed, pct);
  resetDrivePositions();

  FL.spinFor(forward, leftDistance, degrees, false);
  ML.spinFor(forward, leftDistance, degrees, false);
  BL.spinFor(forward, leftDistance, degrees, false);
  FR.spinFor(forward, rightDistance, degrees, false);
  MR.spinFor(forward, rightDistance, degrees, false);
  BR.spinFor(forward, rightDistance, degrees, true);

  StopDriveTrain(hold);
}


void arcTurnRight(int radius, int theta, int velocityunit){
  double goal = (theta*radius)/ratio;

  double leftRadius = radius+(robotWidth/2);
  double rightRadius = radius-(robotWidth/2);
  
  double leftDistance = goal*leftRadius/radius; double leftspeed = velocityunit*(leftRadius/radius);
  double rightDistance = goal*rightRadius/radius; double rightspeed = velocityunit*(rightRadius/radius);
  
  FL.setVelocity(leftspeed, pct);FR.setVelocity(rightspeed, pct);
  ML.setVelocity(leftspeed, pct);MR.setVelocity(rightspeed, pct);
  BL.setVelocity(leftspeed, pct);BR.setVelocity(rightspeed, pct);
  resetDrivePositions();

  FL.spinFor(forward, leftDistance, degrees, false);
  ML.spinFor(forward, leftDistance, degrees, false);
  BL.spinFor(forward, leftDistance, degrees, false);
  FR.spinFor(forward, rightDistance, degrees, false);
  MR.spinFor(forward, rightDistance, degrees, false);
  BR.spinFor(forward, rightDistance, degrees, true);

  StopDriveTrain(hold);
}


void pointTurnRight(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  ML.spinFor(forward, goal, degrees, false);MR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  StopDriveTrain(hold);
}


void pointTurnLeft(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  ML.spinFor(forward, goal, degrees, false);MR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  StopDriveTrain(hold);
}

void sidePivotleft(double theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FR.spinFor(forward, goal, degrees, false);
  MR.spinFor(forward, goal, degrees, false);
  BR.spinFor(forward, goal, degrees, true);
  StopDriveTrain(brake);
}

void sidePivotright(double theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FL.spinFor(forward, goal, degrees, false);
  ML.spinFor(forward, goal, degrees, false);
  BL.spinFor(forward, goal, degrees, true);
  StopDriveTrain(brake);
}


/*---------------------------------------------------------------------------*/
/*-----------------------------Game Object Utility---------------------------*/
/*---------------------------------------------------------------------------*/

void RwingToggle(void){
  RwingCount++;
  if (RwingCount%2==0){
    Rwing.close();
  } else{
    Rwing.open();
  }
}

void LWingToggle(void){
  LwingCount++;
  if (LwingCount%2==0){
    Lwing.close();
  } else{
    Lwing.open();
  }
}