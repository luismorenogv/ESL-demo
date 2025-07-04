// Filename : controller.h 
// Authors : Luis Moreno (s3608255), Luca Provenzano (s3487636)
// Group : 43
// License : N.A. or open source license like LGPL
// Description : Controller wrapper header file
//==============================================================

#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common/xxtypes.h"

/* GLOBAL FUNCTIONS */

// Initializes both the pan and tilt controllers.
void ControllerInitialize(void);

// Steps both controllers forward in time.
void ControllerStep(XXDouble tiltPos, XXDouble tiltDst,
                    XXDouble panPos,  XXDouble panDst,
                    XXDouble dt);

// Terminates both controllers
void ControllerTerminate(void);

// Getter functions for the calculated outputs.
XXDouble getPanOut(void);
XXDouble getTiltOut(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H