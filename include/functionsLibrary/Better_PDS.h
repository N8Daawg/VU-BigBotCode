/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Better_PDS.h                                              */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  Autonomous PIDS utilizing integrators and derivatives     */
/*      See added documentation for the process of creating and tuning        */
/*                                                                            */
/*----------------------------------------------------------------------------*/


/**
 * @brief Upgraded PID that moves the robot in an arc shape
 * @note  cuttently not working. needs tuning and testing
 * 
 * @param radius the radius in inches from the center of the arc to the center of the robot
 * @param theta  the angle, in degrees, of the semi-circle the robot will turn to
 * @param dir    a boolean for the direction of turn (true=left, false=right)
*/
void arcturn2(int radius, double theta, bool dir);


/**
 * @brief Upgraded PID that moves the robot in an straight line
 * @note  cuttently not working. needs tuning and testing
 * 
 * @param desiredPos the distance in inches the robot will travel
*/
void DrivePD2(int desiredPos);

/**
 * @brief Upgraded PID that spins the robot on its axis
 * @note  requires gyroscope
 * @note  cuttently not working. spins indefinetly. needs math work, speed control and tuning,
 * 
 * @param desiredPos the angle in degrees the robot will rotate to
 * @param dir        the direction the robot will rotate
*/
void GyroTurn_PD2(int desiredPos, bool dir);