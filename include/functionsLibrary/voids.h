/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Voids.h                                                   */
/*    Author:       Nathan Beals                                              */
/*    Created:      sat Feb. 18 2023                                          */
/*    Description:  Autonomous voids                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*-----------------------Drivetrain Utility Functions------------------------*/
/*---------------------------------------------------------------------------*/

double getHeading(bool dir);

/**
 * @brief function to get average position of motor group
 * @param front first motor on robot
 * @param back  last motor on robot
 * 
 * @returns average position of motor group
*/
double getPositionAverages(motor front, motor middle, motor back);

/**
 * @brief function to get average velocity of motor group
 * @param front first motor on robot
 * @param back  last motor on robot
 * 
 * @returns average velocity of motor group
*/
double getVelocityAverages(motor front, motor middle, motor back);

/**
 * @brief function to set the average velocity of a drivetrain
 * @param Velocity the velocity to set drivetrain to
 * 
 * @returns none
*/
void SetDriveTrainVelocity(int Velocity);

/**
 * @brief function to reset the positions of a drivetrain
 * 
 * @returns none
*/
void resetDrivePositions();

/**
 * @brief function to stop moving the drivetrain
 * @param Brake the brake type of the drivetrain
 * 
 * @returns none
*/
void StopDriveTrain(brakeType Brake);

/*---------------------------------------------------------------------------*/
/*----------------------------Drivetrain Movements---------------------------*/
/*---------------------------------------------------------------------------*/

/**
 * @brief moves the drivetrain forward
 * @param goal the distance to move in inches
 * @param Velocity the speed to move at
*/
void movefwd(int  goal, int Velocity );

/**
 * @brief moves the drivetrain backwards
 * @param goal the distance to move in inches
 * @param velocityunit the speed to move at
*/
void moverev( int goal, int velocityunit );

/**
 * @brief moves the drivetrain left in an arc
 * @param radius the radius of the turn
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void arcTurnLeft(int radius, int theta, int velocityunit);

/**
 * @brief moves the drivetrain right in an arc
 * @param radius the radius of the turn
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void arcTurnRight(int radius, int theta, int velocityunit);

/**
 * @brief spins the drivetrain right on its axis
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void pointTurnRight(int theta, int velocityunit);

/**
 * @brief spins the drivetrain left on its axis
 * @param theta the degree in degrees of the turn
 * @param velocityunit the speed to move at
*/
void pointTurnLeft(int theta, int velocityunit);

/**
 * @brief old point turn function
*/
void turnleft(int theta, int velocityunit);

/**
 * @brief old point turn function
*/
void turnright(int theta, int velocityunit);

/*---------------------------------------------------------------------------*/
/*-----------------------------Game Object Utility---------------------------*/
/*---------------------------------------------------------------------------*/


/**
 * @brief toggles right wing pneumatics
*/
void RwingToggle(void);

/**
 * @brief toggles left wing pneumatics
*/
void LWingToggle(void);