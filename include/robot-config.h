/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       robot-config.h                                            */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  dexlerations of robot motors and devices                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/
using namespace vex;

extern brain Brain;

// VEXcode devices

// Drivetrain Setup
extern motor FL; extern motor FR;
extern motor ML; extern motor MR;
extern motor BL; extern motor BR;
extern inertial Gyro;

extern pneumatics Rwing;
extern pneumatics Lwing;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
