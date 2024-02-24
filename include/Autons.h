/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Autons.h                                                  */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  file for storing autonamous programs                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "voids.h"
#include "Drive_PDS.h"
#include "VisionVoids.h"
#include "Better_PDS.h"

void test(){
  //DrivePD(50);
  double ang = 0;
  Gyro.resetRotation();
  while (ang>-90){
    ang=Gyro.rotation();
    Brain.Screen.print(ang);
    Brain.Screen.newLine();
    FL.spin(reverse,50,pct);
    ML.spin(reverse,50,pct);
    BL.spin(reverse,50,pct);
    FR.spin(fwd,50,pct);
    MR.spin(fwd,50,pct);
    BR.spin(fwd,50,pct);
    }
    holdDriveTrain();
  
  //reload();
  //before and after distance should be 25sqrt(2) inches
  
}

void match(){

}

void skills(){

}