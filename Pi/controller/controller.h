// controller.h

#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common/xxtypes.h"
#include <stdint.h>

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
void getPanOut(uint16_t* pan_duty, uint8_t* pan_dir);
void getTiltOut(uint16_t* tilt_duty, uint8_t* tilt_dir);
/*XXDouble getPanOut(void);
XXDouble getTiltOut(void);*/

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H