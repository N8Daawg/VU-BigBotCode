/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Better_PDS.h                                              */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  Autonomous PIDS utilizing integrators and derivatives     */
/*      See added documentation for the process of creating and tuning        */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
extern double ratio;
extern double robotWidth;
extern double getPositionAverages(motor front, motor middle, motor back);
extern double getVelocityAverages(motor front, motor middle, motor back);
extern void resetDrivePositions();
extern void StopDriveTrain(brakeType Brake);
extern controller Controller;

/**
 * @brief Upgraded PID that moves the robot in an arc shape
 * @note  cuttently not working. needs tuning and testing
 * 
 * @param radius the radius in inches from the center of the arc to the center of the robot
 * @param theta  the angle, in degrees, of the semi-circle the robot will turn to
 * @param dir    a boolean for the direction of turn (true=left, false=right)
*/
void arcturn2(int radius, double theta, bool dir){
  // 1 degrees = 0.08144868 in
  double desiredPos = (theta/360)*M_PI*radius*2;
  
  double goal = desiredPos/ratio;

  double kp = (goal/80); //0.2// proportional control: gets smaller as error decreases
  double ki = kp*(0.414/0.409); //0.35; // integrater: accumulation of error
  double kd = kp*(0.414/0.18);//0.3; // derivative: change in error
  double ka = 0; // controls how fast the program corrects drift
  
  // status tracking variables
  double error; double prev_Error = 0; 
  double derivative;double integral=0;
  
  double ang; // angle at which the bot has turned 
  double LeftAvg; double RightAvg; double progress;
  double AveRSpeed; double AveLSpeed;
  double RightMotorSpeed; double LeftMotorSpeed; double avgMotorspeed;
  double leftRadius;double rightRadius;
  int loopCount=0;
  int errorCount=0;

  
  resetDrivePositions();
  Gyro.resetRotation();
  double rotation = Gyro.rotation();
  if(dir){ // change radius sides based on turning left or right.
    leftRadius = radius-(robotWidth/2);
    rightRadius = radius+(robotWidth/2);
  } else{
    leftRadius = radius+(robotWidth/2);
    rightRadius = radius-(robotWidth/2);
  }
  while (errorCount<3 && rotation<theta){
    rotation = Gyro.rotation();
    // turn distance of the left track into theta and back to distance of the robot
    LeftAvg = radius*getPositionAverages(FL, ML, BL)/leftRadius;
    RightAvg = radius*getPositionAverages(FR, MR, BR)/rightRadius;
    progress = (LeftAvg + RightAvg)/2; //total distance traveled

    // turn tangential velocity of the left track into rotational velocity and find difference. 
    AveLSpeed = getVelocityAverages(FL, ML, BL)/leftRadius;
    AveRSpeed = getVelocityAverages(FR, MR, BR)/rightRadius;
    ang = (AveRSpeed)-(AveLSpeed);

    // calculate error
    error = goal - progress;
    // calculate integral
    if (loopCount>0){ 
      // create right hand riehmann sum for int
      integral+=(error*0.02);
      // calculate derivative
      derivative = (error-prev_Error)/0.02;
    }
    
    avgMotorspeed = ((error*kp)+(integral*ki)+(derivative*kd));
    if (avgMotorspeed>100){
      avgMotorspeed=100;
    }
    
    
    LeftMotorSpeed = ((avgMotorspeed-(ang*ka))*leftRadius/radius);
    RightMotorSpeed = ((avgMotorspeed+(ang*ka))*rightRadius/radius);


    if(0<LeftMotorSpeed<1)
    LeftMotorSpeed=1;
    if(0<RightMotorSpeed<1)
    RightMotorSpeed=1;
    if (LeftMotorSpeed > -1 && LeftMotorSpeed<0)
    LeftMotorSpeed=-1;
    if (RightMotorSpeed > -1 && RightMotorSpeed<0)
    RightMotorSpeed=-1;

    
    FL.spin(forward, LeftMotorSpeed, pct);
    ML.spin(forward, LeftMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);
    FR.spin(forward, RightMotorSpeed, pct);
    MR.spin(forward, RightMotorSpeed, pct);
    BR.spin(forward, RightMotorSpeed, pct);

    loopCount++;
    prev_Error = error;

    wait(20, msec);
  }
  StopDriveTrain(hold);
}


/**
 * @brief Upgraded PID that moves the robot in an straight line
 * @note  cuttently not working. needs tuning and testing
 * 
 * @param desiredPos the distance in inches the robot will travel
*/
void DrivePD2(int desiredPos){
  // 1 degrees = 0.08144868 in
  
  double goal = desiredPos/ratio;
  double kp = (goal/80); //0.2// proportional control: gets smaller as error decreases
  double ki = kp*(0.414/0.409); //0.35; // integrater: accumulation of error
  double kd = kp*(0.414/0.18);//0.3; // derivative: change in error

  double ka = 0.15; // controls how fast the program corrects drift


  // status tracking variables
  double error; double prev_Error = 0; 
  double derivative;double integral=0;
  double ang; // angle at which the bot has turned 
  double LeftAvg; double RightAvg; double progress;
  double AveRSpeed; double AveLSpeed;
  double RightMotorSpeed; double LeftMotorSpeed; double adjustment;
  int errorCount=0; int loopCount=0;

  goal = desiredPos/ratio;
  double max = (200*84*4*M_PI)/(36*60);

  resetDrivePositions();
  Gyro.resetRotation();

  while(errorCount<3){
    // average the position values
    LeftAvg = getPositionAverages(FL, ML, BL);
    RightAvg = getPositionAverages(FR, MR, BR);
    progress = (LeftAvg + RightAvg)/2;
    ang = Gyro.rotation();
    
    // calculate error
    error = goal - progress;
    
    // calculate integral
    if (loopCount>1){ // create right hand riehmann sum for int
      integral+=(error*0.02);
      // calculate derivative
      derivative = (prev_Error-error)/0.02;
    }

    // adjust motor speeds
    adjustment = (error*kp)+(integral*ki)+(derivative*kd);

    if (adjustment>90){
      adjustment=90;
    }

    LeftMotorSpeed =  adjustment-(ang*ka);
    RightMotorSpeed = adjustment+(ang*ka);

    if(0<LeftMotorSpeed<1)
    LeftMotorSpeed=1;
    if(0<RightMotorSpeed<1)
    RightMotorSpeed=1;
    if (LeftMotorSpeed > -1 && LeftMotorSpeed<0)
    LeftMotorSpeed=-1;
    if (RightMotorSpeed > -1 && RightMotorSpeed<0)
    RightMotorSpeed=-1;

    FL.spin(forward, LeftMotorSpeed, pct);
    ML.spin(forward, LeftMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);
    FR.spin(forward, RightMotorSpeed, pct);
    MR.spin(forward, RightMotorSpeed, pct);
    BR.spin(forward, RightMotorSpeed, pct);

    prev_Error = error;
    loopCount++;

    if (error <5 && error > -5){
      errorCount += 1;
    }
    wait(20, msec);
  }
  StopDriveTrain(hold);
}

/**
 * @brief Upgraded PID that spins the robot on its axis
 * @note  requires gyroscope
 * @note  cuttently not working. spins indefinetly. needs math work, speed control and tuning,
 * 
 * @param desiredPos the angle in degrees the robot will rotate to
 * @param dir        the direction the robot will rotate
*/
void GyroTurn_PD2(int desiredPos, bool dir){
  double kp = (desiredPos/80); //0.2// proportional control: gets smaller as error decreases
  double ki = kp*(0.414/0.409); //0.35; // integrater: accumulation of error
  double kd = kp*(0.414/0.18);//0.3; // derivative: change in error


  double error; // desirec Value - sensor value
  double prev_Error = 0; // Error of last loop ran
  double derivative; // Error - prevError
  double integral=0;
  double adjustedSpeed;

  int errorCount = 0; int loopcount=0;

  resetDrivePositions();
  Gyro.resetRotation();

  while(errorCount<3){
    // calculate error
    error = desiredPos - Gyro.angle();
    
    // calculate derivative
    derivative = (prev_Error-error)/0.02;
    integral +=  error*0.02;
    

    // adjust motor speeds
    adjustedSpeed = (error * kp) + (integral*ki)+(derivative*kd);


    if(0<adjustedSpeed<1)
    adjustedSpeed=1;
    if (adjustedSpeed > -1 && adjustedSpeed<0)
    adjustedSpeed=-1;

    if (dir == true){
      FL.spin(reverse, adjustedSpeed, pct);
      BL.spin(reverse, adjustedSpeed, pct);
      FR.spin(forward, adjustedSpeed, pct);
      BR.spin(forward, adjustedSpeed, pct);
    }
    else {
      FL.spin(forward, adjustedSpeed, pct);
      BL.spin(forward, adjustedSpeed, pct);
      FR.spin(reverse, adjustedSpeed, pct);
      BR.spin(reverse, adjustedSpeed, pct);    
    }

    prev_Error = error;
    loopcount++;
    if (error <5 && error > -5){
      errorCount ++;
    }
    wait(20,msec);
  }
  StopDriveTrain(hold);
}