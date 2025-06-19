// Filename : controller.h 
// Authors : 
// Group : 
// License : N.A. or open source license like LGPL
// Description : description
//==============================================================

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "xxtypes.h"

/* GLOBAL FUNCTIONS*/
void ControllerInitialize(void);
void ControllerStep(XXDouble tiltPos, XXDouble tiltDst, XXDouble panPos, XXDouble panDst);

XXDouble getPanOut(void);
XXDouble getTiltOut(void);

#ifdef __cplusplus
}
#endif

#endif