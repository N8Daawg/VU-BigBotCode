/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Drive_PDs.h                                               */
/*    Author:       Nathan Beals                                              */
/*    Created:      Thu Feb. 23 2023                                          */
/*    Description:  Autonmous codes                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/**
 * @brief simple move forward while other voids are being fixed
 * 
 * @param pos target to turn to in motor degrees
*/
void RunDriveTrain(int pos);

/**
 * @brief simple move forward while other voids are being fixed
 * 
 * @param vel velocity of the robot
*/
void RunFwd(int vel);

/**
 * @brief simple reverse while other voids are being fixed
 * 
 * @param vel velocity of the robot
*/
void RunRev(int vel);

/**
 * @brief simple point turn while other voids are being fixed
 * 
 * @param pos position to turn to
 * @param dir direction to turn
*/
void TurnDriveTrain(int pos, bool dir);

/***
 * @brief a void for autonomous test cases
*/
void test();

/**
 * @brief a void containing match autonomous code
*/
void match();

/**
 * @brief a void containing skills autonomous code
*/
void skills();