/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       VisionConfig.h                                            */
/*    Author:       Nathan Beals                                              */
/*    Created:      Sun Feb. 18 2024                                          */
/*    Description:  declerations of Vision sensor and color signatures        */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
using namespace vex;

vex::vision::signature matchTriball = vex::vision::signature (1, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature redTriball = vex::vision::signature (2, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature BlueTriball = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT1, 50, matchTriball, redTriball, BlueTriball, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/