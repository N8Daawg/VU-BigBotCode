/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       VisionVoids.h                                             */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  Vision Tracking programs                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "VisionConfig.h"
extern void resetDrivePositions();
extern void StopDriveTrain(brakeType Brake);

/**
 * @brief void created to move the robot to an object using a vision sensor
 * @note  old code, not tested. not for current game
 * @note  Created in case we wanted to add a vision sensor
*/
void Vtrack() {
  resetDrivePositions();

  wait(20, msec);

  // VISION//
  int centerFOV = 158;
  int offsetX = 10;

  // SPEED DEFINERS//
  double SpeedL;
  double SpeedR;  

  double scaleL = 1;
  double scaleR = 1;

  while (1) { 

    Vision1.takeSnapshot(matchTriball);    
    Brain.Screen.clearLine();
    wait(30,msec);

    //prior behavior
    //pnumatics.set(false);  

    
    double X = Vision1.largestObject.centerX;
    double Y = Vision1.largestObject.centerY;

    double speed = (-0.000020) *(pow(Y,3)) + 145;

    //double LSpeed = (0.000002 * (pow(X,2))) + 1.5;
    //double RSpeed = (0.000002 * (pow(X,2))) + 0.5;

    //if on the left
    if (X < (centerFOV - offsetX)) {
      scaleR = 1;
      scaleL =  0.56;

    // if on the right
    } else if (X > (centerFOV + offsetX)) {
      scaleL = 1;
      scaleR =  0.56;

    }

    // dead center
    else {
      scaleR = scaleR;
      scaleL = scaleL;
    }

    //________________________//


    // zero scpeed == in claw
    SpeedL = speed*scaleL;
    SpeedR = speed*scaleR;

    FL.spin(fwd, SpeedL, pct);FR.spin(fwd, SpeedR, pct);
    ML.spin(fwd, SpeedL, pct);MR.spin(fwd, SpeedR, pct);
    BL.spin(fwd, SpeedL, pct);BR.spin(fwd, SpeedR, pct);
    
    

    wait(30, msec);
    if ( (Vision1.largestObject.centerY > 185 ) and (((X < (centerFOV - offsetX)) == false) or X > (centerFOV + offsetX)==false )) {
      break;
    }
  }

  StopDriveTrain(coast);
}

/**
 * @brief simple void that turn robot to designated target with vision sensor
 * @note  old code, not tested. not for current game
 * @note  Created in case we wanted to add a vision sensor
*/
void spinToTarget(){
  Vision1.takeSnapshot(matchTriball);
  int x = 0;

  while ((x < 1000) or ((Vision1.largestObject.exists)==false)){
    Vision1.takeSnapshot(matchTriball);

    FL.spin(directionType::fwd, 75, pct);FR.spin(directionType::rev, 75, pct);
    ML.spin(directionType::fwd, 75, pct);MR.spin(directionType::fwd, 75, pct);
    BL.spin(directionType::fwd, 75, pct);BR.spin(directionType::rev, 75, pct);

    wait(20, msec);
  }
  StopDriveTrain(hold);
}
