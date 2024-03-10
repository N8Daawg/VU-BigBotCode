/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Drive_PDs.h                                               */
/*    Author:       Nathan Beals                                              */
/*    Created:      Thu Feb. 23 2023                                          */
/*    Description:  Autonomous PDS                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
extern double ratio;
extern double robotWidth;
extern double getPositionAverages(motor front, motor middle, motor back);
extern double getVelocityAverages(motor front, motor middle, motor back);
extern void resetDrivePositions();
extern void StopDriveTrain(brakeType Brake);

/**
 * @brief  PID that moves the robot in an arc shape
 * @note   Cuttently mostly working. needs slight tuning
 * 
 * @param desiredPos the length of the semi-circle the robot will travel
 * @param dir        a boolean for the direction of turn (true=left, false=right)
*/
void arcturn(int desiredPos, bool dir){ // desiredPOS: distance the robot drives in inches. 360 degrees = 29.3215 inches
// 1 degrees = 0.08144868 in
  
  double goal = desiredPos/ratio;
  double kp = 0.05; // controls how fast the program's rise time 
  double kd = 0.06; // controls how fast the program reacts to approaching the targes
  double ka = 0.15; // controls how fast the program corrects drift
  
  // status tracking variables
  double error; // desired Value - sensor value
  double prev_Error = 0; // Error of last loop ran
  double derivative; // Error - prevError
  double ang; // angle at which the bot has turned 
  double LeftAvg; double RightAvg; double progress;
  double AveRSpeed; double AveLSpeed;
  double RightMotorSpeed; double LeftMotorSpeed; double avgMotorspeed;

  double radius = 2*desiredPos/M_PI; // =DesiredPos/2pi
  double leftRadius;double rightRadius;

  int loopCount=0;
  int errorCount = 0;

  resetDrivePositions();
  Gyro.resetRotation();

  if(dir){ // change radius sides based on turning left or right.
    leftRadius = radius-(robotWidth/2);
    rightRadius = radius+(robotWidth/2);
  } else{
    leftRadius = radius+(robotWidth/2);
    rightRadius = radius-(robotWidth/2);
  }
  while(errorCount<3){
    // turn distance of the left track into theta and back to distance of the robot
    LeftAvg = radius*getPositionAverages(FL,ML,BL)/leftRadius;
    RightAvg = radius*getPositionAverages(FR,MR,BR)/rightRadius;
    progress = (LeftAvg + RightAvg)/2; //total distance traveled
    // turn tangential velocity of the left track into rotational velocity and average out. 
    
    AveLSpeed = getVelocityAverages(FL,ML,BL)/leftRadius;
    AveRSpeed = getVelocityAverages(FR,MR,BR)/rightRadius;
    ang = (AveRSpeed)-(AveLSpeed);

    // calculate error
    error = goal - progress;
    // calculate derivative
    derivative = error - prev_Error;

    // adjust motor speeds
    avgMotorspeed = (error*kp)+(progress*kd); // tangential velocity of the robot

    LeftMotorSpeed = (avgMotorspeed-(ang*ka))*leftRadius/radius;
    RightMotorSpeed = (avgMotorspeed+(ang*ka))*rightRadius/radius;


    if(0<LeftMotorSpeed<1)
    LeftMotorSpeed=1;
    if(0<RightMotorSpeed<1)
    RightMotorSpeed=1;
    if (LeftMotorSpeed > -1 && LeftMotorSpeed<0)
    LeftMotorSpeed=-1;
    if (RightMotorSpeed > -1 && RightMotorSpeed<0)
    RightMotorSpeed=-1;

    
    FL.spin(forward, LeftMotorSpeed, pct);FR.spin(forward, RightMotorSpeed, pct);
    ML.spin(forward, LeftMotorSpeed, pct);MR.spin(forward, RightMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);

    loopCount++;
    prev_Error = error;

    if (error <5 && error > -5){      
      errorCount++;
    }
  }
  //Gyro.setRotation(prevRotaion+Gyro.rotation(), rotationUnits::deg);
  //double totaldistance = (total_progress/loopCount)*ratio;
  Brain.Screen.print(avgMotorspeed);
  holdDriveTrain();

}

/**
 * @brief PID that moves the robot in an straight line
 * @note  mostly working. would like to add integrator
 * 
 * @param desiredPos the distance in inches the robot will travel
*/
void DrivePD(int DesiredPos){
  // control variables
  double kp = 0.15; // controls how fast the program's rise time 
  double kd = 0.15; // controls how fast the program reacts to approaching the targes
  double ka = 0.5; // controls how fast the program corrects drift


  // status tracking variables
  double goal;
  double error; // desirec Value - sensor value
  double prev_Error = 0; // Error of last loop ran
  double derivative; // Error - prevError
  double ang; // angle at which the bot has turned 
  double LeftAvg; double RightAvg; double progress;
  double AveRSpeed; double AveLSpeed;
  double RightMotorSpeed; double LeftMotorSpeed;
  int errorCount=0;

  goal = DesiredPos/ratio;
  resetDrivePositions();
  Gyro.resetRotation();

  while(errorCount<3){
    // average the position values
    LeftAvg = getPositionAverages(FL,ML,BL);
    RightAvg = getPositionAverages(FR,MR,BR);
    progress = (LeftAvg + RightAvg)/2;
    //ang = Gyro.rotation();

    // find the difference in motor speeds (if no gyro sensor)
    AveLSpeed = getVelocityAverages(FL,ML,BL);
    AveRSpeed = getVelocityAverages(FR,MR,BR);
    ang = (AveRSpeed)-(AveLSpeed);

    // calculate error
    error = goal - progress;
    
    // calculate derivative
    derivative = error - prev_Error;

    // adjust motor speeds
    LeftMotorSpeed = (error*kp) + (progress*kd) -(ang*ka);
    RightMotorSpeed = (error*kp) + (progress*kd) +(ang*ka);


    if(0<LeftMotorSpeed<1)
    LeftMotorSpeed=1;
    if(0<RightMotorSpeed<1)
    RightMotorSpeed=1;
    if (LeftMotorSpeed > -1 && LeftMotorSpeed<0)
    LeftMotorSpeed=-1;
    if (RightMotorSpeed > -1 && RightMotorSpeed<0)
    RightMotorSpeed=-1;

    FL.spin(forward, LeftMotorSpeed, pct);FR.spin(forward, RightMotorSpeed, pct);
    ML.spin(forward, LeftMotorSpeed, pct);MR.spin(forward, RightMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);

    prev_Error = error;

    if (error <5 && error > -5){
      errorCount += 1;
    }
  }
  holdDriveTrain();
}

/**
 * @brief PID that moves the robot in an straight line
 * @note  mostly working. better to use drivePD with negative distance
 * 
 * @param desiredPos the distance in inches the robot will travel
*/
void reversePD(int DesiredPos){
  // control variables
  double kp = 0.15; // controls how fast the program's rise time 
  double kd = 0.05; // controls how fast the program reacts to approaching the targes
  double ka = 0.5; // controls how fast the program corrects drift


  // status tracking variables
  double goal;
  double error; // desirec Value - sensor value
  double prev_Error = 0; // Error of last loop ran
  double derivative; // Error - prevError
  double ang; // angle at which the bot has turned 
  double LeftAvg; double RightAvg; double progress;
  double AveRSpeed; double AveLSpeed;
  double RightMotorSpeed; double LeftMotorSpeed;
  int errorCount=0;

  resetDrivePositions();
  Gyro.resetRotation(); 

  while(errorCount<3){
    // average the position values
    LeftAvg = getPositionAverages(FL,ML,BL);
    RightAvg = getPositionAverages(FR,MR,BR);

    progress = (LeftAvg + RightAvg)/2;
    ang = Gyro.rotation(); 

    // find the difference in motor speeds (if no gyro sensor)
    // AveLSpeed = getVelocityAverages(FL, BL);
    // AveRSpeed = getVelocityAverages(FR, BR);
    // ang = (AveRSpeed)-(AveLSpeed);

  
    // calculate error
    error = (DesiredPos + progress);
    // calculate derivative
    derivative = error - prev_Error;

    // adjust motor speeds
    LeftMotorSpeed = (error * kp) + (derivative*kd) - (ang* ka);
    RightMotorSpeed = (error * kp) + (derivative*kd) + (ang* ka);

    if(0<LeftMotorSpeed<1)
    LeftMotorSpeed=1;
    if(0<RightMotorSpeed<1)
    RightMotorSpeed=1;
    if (LeftMotorSpeed > -1 && LeftMotorSpeed<0)
    LeftMotorSpeed=-1;
    if (RightMotorSpeed > -1 && RightMotorSpeed<0)
    RightMotorSpeed=-1;

    FL.spin(reverse, LeftMotorSpeed, pct);FR.spin(reverse, RightMotorSpeed, pct);
    ML.spin(reverse, LeftMotorSpeed, pct);MR.spin(reverse, RightMotorSpeed, pct);
    BL.spin(reverse, LeftMotorSpeed, pct);BR.spin(reverse, RightMotorSpeed, pct);

    prev_Error = error;

    if (error <5 && error > -5){
      errorCount ++;
    }
  }
  holdDriveTrain();
}

/**
 * @brief PID spins the robot on its axis
 * @note  requires gyroscope
 * @note  cuttently not working. spins indefinetly. needs math work, speed control and tuning,
 * 
 * @param desiredPos the angle in degrees the robot will rotate to
 * @param dir        the direction the robot will rotate
*/
void GyroTurn_PD(int desiredPos, bool dir){
  double kp = 0.15; // controls how fast the program's rise time 
  double kd = 0.05; // controls how fast the program reacts to approaching the targes


  double error; // desirec Value - sensor value
  double prev_Error = 0; // Error of last loop ran
  double derivative; // Error - prevError
  double LeftMotorSpeed;
  double RightMotorSpeed;

  int errorCount = 0;

  resetDrivePositions();
  Gyro.resetRotation();

  while(errorCount<3){
    // calculate error
    error = desiredPos - Gyro.angle();
    
    // calculate derivative
    derivative = error - prev_Error;

    // adjust motor speeds
    LeftMotorSpeed = (error * kp) + (derivative*kd);
    RightMotorSpeed = (error * kp) + (derivative*kd);


    if(0<LeftMotorSpeed<1)
    LeftMotorSpeed=1;
    if(0<RightMotorSpeed<1)
    RightMotorSpeed=1;
    if (LeftMotorSpeed > -1 && LeftMotorSpeed<0)
    LeftMotorSpeed=-1;
    if (RightMotorSpeed > -1 && RightMotorSpeed<0)
    RightMotorSpeed=-1;

    if (dir == true){
      FL.spin(reverse, LeftMotorSpeed, pct);FR.spin(forward, RightMotorSpeed, pct);
      ML.spin(reverse, LeftMotorSpeed, pct);MR.spin(forward, RightMotorSpeed, pct);
      BL.spin(reverse, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);
    }
    else {
      FL.spin(forward, LeftMotorSpeed, pct);FR.spin(reverse, RightMotorSpeed, pct);
      ML.spin(forward, LeftMotorSpeed, pct);MR.spin(reverse, RightMotorSpeed, pct);
      BL.spin(forward, LeftMotorSpeed, pct);BR.spin(reverse, RightMotorSpeed, pct); 
    }

    prev_Error = error;

    if (error <5 && error > -5){
      errorCount ++;
    }
  }
  holdDriveTrain();
}