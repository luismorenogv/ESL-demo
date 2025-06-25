// Filename : controller.h
// Description : A clean C wrapper for the 20-sim generated pan/tilt controllers.

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

// Terminates both controllers (frees memory, etc.).
void ControllerTerminate(void);

// Getter functions for the calculated outputs.
XXDouble getPanOut(void);
XXDouble getTiltOut(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H