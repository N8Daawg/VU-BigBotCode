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
    LeftAvg = radius*getPositionAverages(FL, FML, BML, BL)/leftRadius;
    RightAvg = radius*getPositionAverages(FR, FMR, BMR, BR)/rightRadius;
    progress = (LeftAvg + RightAvg)/2; //total distance traveled

    // turn tangential velocity of the left track into rotational velocity and find difference. 
    AveLSpeed = getVelocityAverages(FL, FML, BML, BL)/leftRadius;
    AveRSpeed = getVelocityAverages(FR, FMR, BMR, BR)/rightRadius;
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

    
    FL.spin(forward, LeftMotorSpeed, pct);FR.spin(forward, RightMotorSpeed, pct);
    FML.spin(forward, LeftMotorSpeed, pct);FMR.spin(forward, RightMotorSpeed, pct);
    BML.spin(forward, LeftMotorSpeed, pct);BMR.spin(forward, RightMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);

    loopCount++;
    prev_Error = error;

    wait(20, msec);
  }
  StopDriveTrain(hold);
}


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
    LeftAvg = getPositionAverages(FL, FML, BML, BL);
    RightAvg = getPositionAverages(FR, FMR, BMR, BR);
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

    FL.spin(forward, LeftMotorSpeed, pct);FR.spin(forward, RightMotorSpeed, pct);
    FML.spin(forward, LeftMotorSpeed, pct);FMR.spin(forward, RightMotorSpeed, pct);
    BML.spin(forward, LeftMotorSpeed, pct);BMR.spin(forward, RightMotorSpeed, pct);
    BL.spin(forward, LeftMotorSpeed, pct);BR.spin(forward, RightMotorSpeed, pct);

    prev_Error = error;
    loopCount++;

    if (error <5 && error > -5){
      errorCount += 1;
    }
    wait(20, msec);
  }
  StopDriveTrain(hold);
}


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
    error = desiredPos - getHeading(dir);
    
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
      FL.spin(reverse, adjustedSpeed, pct);FR.spin(forward, adjustedSpeed, pct);
      FML.spin(reverse, adjustedSpeed, pct);FMR.spin(forward, adjustedSpeed, pct);
      BML.spin(reverse, adjustedSpeed, pct);BMR.spin(forward, adjustedSpeed, pct);
      BL.spin(reverse, adjustedSpeed, pct);BR.spin(forward, adjustedSpeed, pct);
    }
    else {
      FL.spin(forward, adjustedSpeed, pct);FR.spin(reverse, adjustedSpeed, pct);
      FML.spin(forward, adjustedSpeed, pct);FMR.spin(reverse, adjustedSpeed, pct);
      BML.spin(forward, adjustedSpeed, pct);BMR.spin(reverse, adjustedSpeed, pct);
      BL.spin(forward, adjustedSpeed, pct);BR.spin(reverse, adjustedSpeed, pct); 
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