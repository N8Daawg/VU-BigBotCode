/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sat Feb 18 2023                                           */
/*    Description:  PortaLib 2022-2023 Basic-Advanced program examples        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller           controller             
//
// FL                   Motor         1        
// ML                   Morot         2
// BL                   Motor         3        
// FR                   Motor         4
// MR                   Motor         5        
// BR                   Motor         6        
// Gyro                 Gyroscope     7        
//
// wings                pneumatics    H
// ---- END VEXCODE CONFIGURED DEVICES ----
// this is test #2 of the remote repository function

#include "vex.h"
#include "Autons.h"

using namespace vex;

// A global instance of competition
competition Competition;

// A global instance of user controller
controller Controller;


// define your global instances of motors and other devices here
int deadzone = 10;
double ratio = (5.6*M_PI)/360;
double robotWidth=13.5;
int RwingCount = 0;
int LwingCount = 0;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void autonomous(void) {
  // ..........................................................................
  test();
  // ..........................................................................
}


/*---------------------------------------------------------------------------*/
/*                          Pre-User Control Functions                       */
/*                                                                           */
/*  A space used for defining certain usercontrol voids for button control.  */
/*  Do them in the following function.  You must return from this function   */
/*  or the usercontrol and autonimous tasks will not be started.  This       */
/*  space is only for user voids and not autonimous functions.               */
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double leftPower;
double rightPower;
void usercontrol(void) {
  Controller.ButtonX.pressed(RwingToggle);
  Controller.ButtonY.pressed(LWingToggle);
  while (1) {
    /*---------------------------------------------------------------------------*/
    /*-----------------------------CONTROLLER MAP--------------------------------*/
    /*---------------------------------------------------------------------------*/
    //creating a deadzone on the controller to solve motor drift issues
    if(Controller.Axis1.position() <  deadzone && Controller.Axis1.position() < -deadzone &&
       Controller.Axis2.position() <  deadzone && Controller.Axis2.position() < -deadzone &&
       Controller.Axis3.position() <  deadzone && Controller.Axis3.position() < -deadzone &&
       Controller.Axis4.position() <  deadzone && Controller.Axis4.position() < -deadzone ) {
        FL.stop(coast);ML.stop(coast);BL.stop(coast);
        FR.stop(coast);MR.stop(coast);BR.stop(coast);
      }
    else {
      leftPower = Controller.Axis4.position() - Controller.Axis3.position();
      rightPower = Controller.Axis2.position() + Controller.Axis1.position();
      //basic movement map
      FL.spin(fwd, leftPower, pct);FR.spin(fwd, rightPower, pct);
      ML.spin(fwd, leftPower, pct);MR.spin(fwd, rightPower, pct);
      BL.spin(fwd, leftPower, pct);BR.spin(fwd, rightPower, pct);
      }

    /*---------------------------------------------------------------------------*/
    /*-----------------------------PNEUMATICS MAPPING-------------------------------*/
    /*---------------------------------------------------------------------------*/

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
