/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Drive_PDs.h                                               */
/*    Author:       Nathan Beals                                              */
/*    Created:      Thu Feb. 23 2023                                          */
/*    Description:  Autonomous PDS                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief  PID that moves the robot in an arc shape
 * @note   Cuttently mostly working. needs slight tuning
 * 
 * @param desiredPos the length of the semi-circle the robot will travel
 * @param dir        a boolean for the direction of turn (true=left, false=right)
*/
void arcturn(int desiredPos, bool dir);

/**
 * @brief PID that moves the robot in an straight line
 * @note  mostly working. would like to add integrator
 * 
 * @param desiredPos the distance in inches the robot will travel
*/
void DrivePD(int DesiredPos);

/**
 * @brief PID that moves the robot in an straight line
 * @note  mostly working. better to use drivePD with negative distance
 * 
 * @param desiredPos the distance in inches the robot will travel
*/
void reversePD(int DesiredPos);

/**
 * @brief PID spins the robot on its axis
 * @note  requires gyroscope
 * @note  cuttently not working. spins indefinetly. needs math work, speed control and tuning,
 * 
 * @param desiredPos the angle in degrees the robot will rotate to
 * @param dir        the direction the robot will rotate
*/
void GyroTurn_PD(int desiredPos, bool dir);