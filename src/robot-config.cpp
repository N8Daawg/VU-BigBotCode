#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// VEXcode device constructors

// DriveTrain Set up
motor FL = motor(PORT16, ratio18_1, true);
motor ML = motor(PORT12, ratio18_1, true);
motor BL = motor(PORT19, ratio18_1, true);
motor FR = motor(PORT7, ratio18_1, false);
motor MR = motor(PORT13, ratio18_1, false);
motor BR = motor(PORT20, ratio18_1, false);
inertial Gyro = inertial(PORT5);


// Wall Setup
pneumatics Rwing = pneumatics(Brain.ThreeWirePort.H);
pneumatics Lwing = pneumatics(Brain.ThreeWirePort.G);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
  Rwing.close();
  Lwing.close();
}
