// Filename : controller.h 
// Authors : 
// Group : 
// License : N.A. or open source license like LGPL
// Description : description
//==============================================================

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "xxtypes.h"

/* GLOBAL FUNCTIONS*/
void ControllerInitialize(void);
void ControllerStep(XXDouble tiltPos, XXDouble tiltDst, XXDouble panPos, XXDouble panDst);

XXDouble getPanOut(void);
XXDouble getTiltOut(void);


#endif