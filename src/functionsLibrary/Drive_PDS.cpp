/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Drive_PDs.h                                               */
/*    Author:       Nathan Beals                                              */
/*    Created:      Thu Feb. 23 2023                                          */
/*    Description:  Autonomous PDS                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"


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
    LeftAvg = radius*getPositionAverages(FL,FML,BML,BL)/leftRadius;
    RightAvg = radius*getPositionAverages(FR,FMR,BMR,BR)/rightRadius;
    progress = (LeftAvg + RightAvg)/2; //total distance traveled
    // turn tangential velocity of the left track into rotational velocity and average out. 
    
    AveLSpeed = getVelocityAverages(FL,FML,BML,BL)/leftRadius;
    AveRSpeed = getVelocityAverages(FR,FMR,BMR,BR)/rightRadius;
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
    FML.spin(forward, LeftMotorSpeed, pct);FMR.spin(forward, RightMotorSpeed, pct);
    BML.spin(forward, LeftMotorSpeed, pct);BMR.spin(forward, RightMotorSpeed, pct);
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
  StopDriveTrain(hold);

}


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
    LeftAvg = getPositionAverages(FL,FML,BML,BL);
    RightAvg = getPositionAverages(FR,FMR,BMR,BR);
    progress = (LeftAvg + RightAvg)/2;
    //ang = Gyro.rotation();

    // find the difference in motor speeds (if no gyro sensor)
    AveLSpeed = getVelocityAverages(FL,FML,BML,BL);
    AveRSpeed = getVelocityAverages(FR,FMR,BMR,BR);
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
    FML.spin(forward, LeftMotorSpeed, pct);FMR.spin(forward, RightMotorSpeed, pct);
    BML.spin(forward, LeftMotorSpeed, pct);BMR.spin(forward, RightMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);

    prev_Error = error;

    if (error <5 && error > -5){
      errorCount += 1;
    }
  }
  StopDriveTrain(hold);
}


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
    LeftAvg = getPositionAverages(FL,FML,BML,BL);
    RightAvg = getPositionAverages(FR,FMR,BML,BR);

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
    FML.spin(reverse, LeftMotorSpeed, pct);FMR.spin(reverse, RightMotorSpeed, pct);
    BML.spin(reverse, LeftMotorSpeed, pct);BMR.spin(reverse, RightMotorSpeed, pct);
    BL.spin(reverse, LeftMotorSpeed, pct);BR.spin(reverse, RightMotorSpeed, pct);

    prev_Error = error;

    if (error <5 && error > -5){
      errorCount ++;
    }
  }
  StopDriveTrain(hold);
}


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
    error = desiredPos - getHeading(dir);
    
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
      FML.spin(reverse, LeftMotorSpeed, pct);FMR.spin(forward, RightMotorSpeed, pct);
      BML.spin(reverse, LeftMotorSpeed, pct);BMR.spin(forward, RightMotorSpeed, pct);
      BL.spin(reverse, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);
    }
    else {
      FL.spin(forward, LeftMotorSpeed, pct);FR.spin(reverse, RightMotorSpeed, pct);
      FML.spin(forward, LeftMotorSpeed, pct);FMR.spin(reverse, RightMotorSpeed, pct);
      BML.spin(forward, LeftMotorSpeed, pct);BMR.spin(reverse, RightMotorSpeed, pct);
      BL.spin(forward, LeftMotorSpeed, pct);BR.spin(reverse, RightMotorSpeed, pct); 
    }

    prev_Error = error;

    if (error <5 && error > -5){
      errorCount ++;
    }
  }
  StopDriveTrain(hold);
}