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
#include <math.h>
#include <stdio.h>

#define TILT_CONVERGENCE 0.06
#define PAN_CONVERGENCE 0.15

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

void ControllerStep(XXDouble tiltPos, XXDouble tiltDst, XXDouble panPos, XXDouble panDst)
{
    printf("tilt Error: %.2f\n", tiltPos - tiltDst);
    //printf("pan Error: %.2f\n", panPos - panDst);
    // tiltDst = tiltPos;
    if(fabs(tiltPos - tiltDst) < TILT_CONVERGENCE) tiltDst = tiltPos;
    if(fabs(panPos - panDst) < PAN_CONVERGENCE) panDst = panPos;
        //set pan inputs
    SetPanInputs(panPos, panDst);

    //calculate pan dynamic
    PanCalculateDynamic();
    PanCalculateOutput();

    //read pan output
    ReadPanOutputs();

    //set tilt inputs
    SetTiltInputs(tiltPos, tiltDst, corr);

    //calculate tilt dynamic
    TiltCalculateDynamic();
    TiltCalculateOutput();

    //read pan output
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