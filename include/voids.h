/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Voids.h                                                   */
/*    Author:       Nathan Beals                                              */
/*    Created:      sat Feb. 18 2023                                          */
/*    Description:  Autonimous voids                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
extern double ratio;
extern double robotWidth;

/*---------------------------------------------------------------------------*/
/*-----------------------Drivetrain Utility Functions------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief function to get average position of motor group
 * @param front first motor on robot
 * @param back  last motor on robot
 * 
 * @returns average position of motor group
*/
double getPositionAverages(motor front, motor middle, motor back){
  return (front.position(degrees)+middle.position(degrees)+back.position(degrees))/3;
}

/**
 * @brief function to get average velocity of motor group
 * @param front first motor on robot
 * @param back  last motor on robot
 * 
 * @returns average velocity of motor group
*/
double getVelocityAverages(motor front, motor middle, motor back){
  return (front.velocity(pct)+middle.velocity(pct)+back.velocity(pct))/3;
}

/**
 * @brief function to set the average velocity of a drivetrain
 * @param Velocity the velocity to set drivetrain to
 * 
 * @returns none
*/
void SetDriveTrainVelocity(int Velocity) {
  FL.setVelocity(Velocity, pct);FR.setVelocity(Velocity, pct);
  ML.setVelocity(Velocity, pct);MR.setVelocity(Velocity, pct);
  BL.setVelocity(Velocity, pct);BR.setVelocity(Velocity, pct);
}

/**
 * @brief function to reset the positions of a drivetrain
 * 
 * @returns none
*/
void resetDrivePositions(){
  FL.resetPosition();FR.resetPosition();
  ML.resetPosition();MR.resetPosition();
  BL.resetPosition();BR.resetPosition();
}

/**
 * @brief function to stop moving the drivetrain
 * @param Brake the brake type of the drivetrain
 * 
 * @returns none
*/
void StopDriveTrain(brakeType Brake){
  FL.stop(Brake);
  BL.stop(Brake);
  FR.stop(Brake);
  BR.stop(Brake);
}

/*---------------------------------------------------------------------------*/
/*----------------------------Drivetrain Movements---------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief moves the drivetrain forward
 * @param goal the distance to move in inches
 * @param Velocity the speed to move at
*/
void movefwd(int  goal, int Velocity ){
  SetDriveTrainVelocity(Velocity);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(forward, theta, degrees, false);FR.spinFor(forward, theta, degrees,  false);
  ML.spinFor(forward, theta, degrees, false);MR.spinFor(forward, theta, degrees,  false);
  BL.spinFor(forward, theta, degrees, false);BR.spinFor(forward, theta, degrees,  true);
  
  StopDriveTrain(brake);
}

/**
 * @brief moves the drivetrain backwards
 * @param goal the distance to move in inches
 * @param velocityunit the speed to move at
*/
void moverev( int goal, int velocityunit ){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();
  double theta = goal/ratio;

  FL.spinFor(reverse, theta, degrees, false);FR.spinFor(reverse, theta, degrees,  false);
  ML.spinFor(reverse, theta, degrees, false);MR.spinFor(reverse, theta, degrees,  false);
  BL.spinFor(reverse, theta, degrees, false);BR.spinFor(reverse, theta, degrees,  true);
  
  StopDriveTrain(brake);
}

/**
 * @brief moves the drivetrain left in an arc
 * @param radius the radius of the turn
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void arcTurnLeft(int radius, int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double leftDistance = theta * (radius-(robotWidth/2))/4;
  double rightDistance = theta * (radius+(robotWidth/2))/4;

  //leftDistance = leftDistance/ratio;rightDistance = rightDistance/ratio;
  
  FL.spinFor(forward, leftDistance, degrees, false);
  ML.spinFor(forward, leftDistance, degrees, false);
  BL.spinFor(forward, leftDistance, degrees, false);
  FR.spinFor(forward, rightDistance, degrees, false);
  MR.spinFor(forward, rightDistance, degrees, false);
  BR.spinFor(forward, rightDistance, degrees, true);

  StopDriveTrain(hold);
}

/**
 * @brief moves the drivetrain right in an arc
 * @param radius the radius of the turn
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void arcTurnRight(int radius, int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double leftDistance = theta * (radius+(robotWidth/2))/4;
  double rightDistance = theta * (radius-(robotWidth/2))/4;

  //leftDistance = leftDistance/ratio;rightDistance = rightDistance/ratio;
  
  FL.spinFor(forward, leftDistance, degrees, false);
  ML.spinFor(forward, leftDistance, degrees, false);
  BL.spinFor(forward, leftDistance, degrees, false);
  FR.spinFor(forward, rightDistance, degrees, false);
  MR.spinFor(forward, rightDistance, degrees, false);
  BR.spinFor(forward, rightDistance, degrees, true);

  StopDriveTrain(hold);
}

/**
 * @brief spins the drivetrain right on its axis
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void pointTurnRight(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/2;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  ML.spinFor(forward, goal, degrees, false);MR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  
  StopDriveTrain(hold);
}

/**
 * @brief spins the drivetrain left on its axis
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void pointTurnLeft(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  double goal = theta*(robotWidth/2)/2;

  FL.spinFor(forward, goal, degrees, false);FR.spinFor(reverse, goal, degrees,  false);
  ML.spinFor(forward, goal, degrees, false);MR.spinFor(reverse, goal, degrees,  false);
  BL.spinFor(forward, goal, degrees, false);BR.spinFor(reverse, goal, degrees,  true);
  
  StopDriveTrain(hold);
}

/**
 * @brief old point turn function
 * @note obsolete
*/
void turnleft(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  FL.spinFor(reverse, theta, degrees, false);FR.spinFor(forward, theta, degrees,  false);
  ML.spinFor(reverse, theta, degrees, false);MR.spinFor(forward, theta, degrees,  false);
  BL.spinFor(reverse, theta, degrees, false);BR.spinFor(forward, theta, degrees,  true);
  
  StopDriveTrain(brake);
}

/**
 * @brief old point turn function
 * @note obsolete
*/
void turnright(int theta, int velocityunit){
  SetDriveTrainVelocity(velocityunit);
  resetDrivePositions();

  FL.spinFor(forward, theta, degrees, false);FR.spinFor(reverse, theta, degrees,  false);
  ML.spinFor(forward, theta, degrees, false);MR.spinFor(reverse, theta, degrees,  false);
  BL.spinFor(forward, theta, degrees, false);BR.spinFor(reverse, theta, degrees,  true);
  
  StopDriveTrain(hold);
}


/*---------------------------------------------------------------------------*/
/*-----------------------------Game Object Utility---------------------------*/
/*---------------------------------------------------------------------------*/

extern int RwingCount;
/**
 * @brief toggles right wing pneumatics
*/
void RwingToggle(void){
  RwingCount++;
  if (RwingCount%2==0){
    Rwing.close();
  } else{
    Rwing.open();
  }
}

extern int LwingCount;
/**
 * @brief toggles left wing pneumatics
*/
void LWingToggle(void){
  LwingCount++;
  if (LwingCount%2==0){
    Lwing.close();
  } else{
    Lwing.open();
  }
}