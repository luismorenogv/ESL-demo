// Filename : controller.c 
// Authors :
// Group :
// License : N.A. or open source license like LGPL
// Description : 
//==============================================================


/** INCLUDES **/
#include "controller.h"
#include "pan_model.h"
#include "tilt_model.h"

/* EXTERN VARIABLES */
extern XXDouble pan_V[];
extern XXDouble tilt_V[];

/** LOCAL VARIABLES **/
XXDouble  panOut = 0.0, 
         tiltOut = 0.0,
            corr = 0.0;

/** LOCAL FUNCTIONS **/
void SetPanInputs(XXDouble pos, XXDouble dst);
void ReadPanOutputs(void);
void SetTiltInputs(XXDouble pos, XXDouble dst, XXDouble corr);
void ReadTiltOutputs(void);

void ControllerInitialize(void)
{
    //Pan init
    PanModelInitialize();
    PanCalculateInitial();
    //Tilt init
    TiltModelInitialize();
    TiltCalculateInitial();
}

void ControllerStep(XXDouble tiltPos, XXDouble tiltDst, XXDouble panPos, XXDouble panDst, XXDouble dt)
{
    // Update the global step_size variable in each model before calculation
    pan_step_size = dt;
    tilt_step_size = dt;

    SetPanInputs(panPos, panDst);
    PanCalculateStatic(); PanCalculateInput(); PanCalculateDynamic();

    for(int i = 0; i < pan_states_size; i++) { pan_s[i] += pan_step_size * pan_R[i]; }
    PanCalculateOutput();
    ReadPanOutputs();

    SetTiltInputs(tiltPos, tiltDst, corr);
    TiltCalculateStatic(); TiltCalculateInput(); TiltCalculateDynamic();
    
    for(int i = 0; i < tilt_states_size; i++) { tilt_s[i] += tilt_step_size * tilt_R[i]; }
    TiltCalculateOutput();
    ReadTiltOutputs();
}


void SetPanInputs(XXDouble pos, XXDouble dst)
{
    pan_V[7] = dst;
    pan_V[8] = pos;
}

void ReadPanOutputs(void)
{
    corr    = pan_V[6];
    panOut  = pan_V[9];
}

void SetTiltInputs(XXDouble pos, XXDouble dst, XXDouble corr)
{
    tilt_V[8]  = corr;
    tilt_V[9]  = dst;
    tilt_V[10] = pos;
}

void ReadTiltOutputs(void)
{
    tiltOut = tilt_V[11];
}

XXDouble getPanOut(void){return panOut;}
XXDouble getTiltOut(void){return tiltOut;}