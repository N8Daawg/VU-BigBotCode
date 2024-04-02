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


double getPositionAverages(motor front, motor Fmiddle, motor Bmiddle, motor back){
  return (front.position(degrees)+Fmiddle.position(degrees)+Bmiddle.position(degrees)+back.position(degrees))/4;
}


double getVelocityAverages(motor front, motor Fmiddle, motor Bmiddle, motor back){
  return (front.velocity(pct)+Fmiddle.velocity(pct)+Bmiddle.velocity(pct)+back.velocity(pct))/4;
}


void SetDriveTrainVelocity(int Velocity) {
  FL.setVelocity(Velocity, pct);FR.setVelocity(Velocity, pct);
  FML.setVelocity(Velocity, pct);FMR.setVelocity(Velocity, pct);
  BML.setVelocity(Velocity, pct);BMR.setVelocity(Velocity, pct);
  BL.setVelocity(Velocity, pct);BR.setVelocity(Velocity, pct);
}


void resetDrivePositions(){
  FL.resetPosition();FR.resetPosition();
  FML.resetPosition();FMR.resetPosition();
  BML.resetPosition();BMR.resetPosition();
  BL.resetPosition();BR.resetPosition();
}


void StopDriveTrain(brakeType Brake){
  FL.stop(Brake);FR.stop(Brake);
  FML.stop(Brake);FMR.stop(Brake);
  BML.stop(Brake);BMR.stop(Brake);
  BL.stop(Brake);BR.stop(Brake);
}

/*---------------------------------------------------------------------------*/
/*----------------------------Drivetrain Movements---------------------------*/
/*---------------------------------------------------------------------------*/


void movefwd(int  goal, int Velocity ){
  SetDriveTrainVelocity(Velocity);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(forward, theta, degrees, false);FR.spinFor(forward, theta, degrees,  false);
  FML.spinFor(forward, theta, degrees, false);FMR.spinFor(forward, theta, degrees,  false);
  BML.spinFor(forward, theta, degrees, false);BMR.spinFor(forward, theta, degrees,  false);
  BL.spinFor(forward, theta, degrees, false);BR.spinFor(forward, theta, degrees,  true);
  
  StopDriveTrain(brake);
}


void moverev( int goal, int velocityunit ){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(reverse, theta, degrees, false);FR.spinFor(reverse, theta, degrees,  false);
  FML.spinFor(reverse, theta, degrees, false);FMR.spinFor(reverse, theta, degrees,  false);
  BML.spinFor(reverse, theta, degrees, false);BMR.spinFor(reverse, theta, degrees,  false);
  BL.spinFor(reverse, theta, degrees, false);BR.spinFor(reverse, theta, degrees,  true);
  
  StopDriveTrain(brake);
}


void arcTurnLeft(int radius, int theta, int velocityunit){
  double goal = (theta*radius)/ratio;

  double leftRadius = radius-(robotWidth/2);
  double rightRadius = radius+(robotWidth/2);
  
  double leftDistance = goal*leftRadius/radius; double leftspeed = velocityunit*(leftRadius/radius);
  double rightDistance = goal*rightRadius/radius; double rightspeed = velocityunit*(rightRadius/radius);
  
  resetDrivePositions();
  FL.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);FR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, false);
  FML.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);FMR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, false);
  BML.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);BMR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, false);
  BL.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);BR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, true);

  StopDriveTrain(hold);
}


void arcTurnRight(int radius, int theta, int velocityunit){
  double goal = (theta*radius)/ratio;

  double leftRadius = radius+(robotWidth/2);
  double rightRadius = radius-(robotWidth/2);
  
  double leftDistance = goal*leftRadius/radius; double leftspeed = velocityunit*(leftRadius/radius);
  double rightDistance = goal*rightRadius/radius; double rightspeed = velocityunit*(rightRadius/radius);
  
  resetDrivePositions();
  FL.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);FR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, false);
  FML.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);FMR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, false);
  BML.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);BMR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, false);
  BL.spinFor(forward, leftDistance, degrees, leftspeed, velocityUnits::pct, false);BR.spinFor(forward, rightDistance, degrees, rightspeed, velocityUnits::pct, true);
  StopDriveTrain(hold);
}


void pointTurnRight(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  FML.spinFor(forward, goal, degrees, false);FMR.spinFor(reverse, goal, degrees,  false);
  BML.spinFor(forward, goal, degrees, false);BMR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  StopDriveTrain(hold);
}


void pointTurnLeft(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  FML.spinFor(forward, goal, degrees, false);FMR.spinFor(reverse, goal, degrees,  false);
  BML.spinFor(forward, goal, degrees, false);BMR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  StopDriveTrain(hold);
}

void sidePivotleft(double theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FR.spinFor(forward, goal, degrees, false);
  FMR.spinFor(forward, goal, degrees, false);
  BMR.spinFor(forward, goal, degrees, false);
  BR.spinFor(forward, goal, degrees, true);
  StopDriveTrain(brake);
}

void sidePivotright(double theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/ratio;

  FL.spinFor(forward, goal, degrees, false);
  FML.spinFor(forward, goal, degrees, false);
  BML.spinFor(forward, goal, degrees, false);
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