/*
 * SDOTPRODUCT S-Function to compute dot product (multiply-accumulate)
 *      of two real or complex vectors
 *
 *  D. Orofino, 12-97
 *  D. Boghiu,  03-98
 *  Copyright 1990-2009 The MathWorks, Inc.
 */

#define S_FUNCTION_NAME MATLink
#define S_FUNCTION_LEVEL 2

#include <simstruc.h>
#include "mavros/mavros.h"
#include <iostream>

#define NUM_PARAMS (0)

static void mdlInitializeSizes(SimStruct *S)
{
    // sets the number of parameters that the S-Function Has.
    // For now, we don't have any, but in the future, we might need some.
    // Look at the the template file for guidance on how to deal with parameters
    // passed to the function
    ssSetNumSFcnParams(S,  NUM_PARAMS);
    // check to make sure that the right number of parameters were passed
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }

    /***************************************************/
    /* Take Care of the Continuous and Discrete States */
    /***************************************************/
    ssSetNumContStates(S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(S, 0);   /* number of discrete states             */



    /*****************************/
    /* Configure the input ports */
    /*****************************/
    if(!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 21);
    ssSetInputPortDirectFeedThrough(S, 0, 1); // we will use the input in the output step

    /******************************/
    /* Configure the output ports */
    /******************************/
    if (!ssSetNumOutputPorts(S, 1)) return;
    if(!ssSetOutputPortVectorDimension(S, 0, 1)) return;
    ssSetOutputPortWidth(S, 0, 21); // set the output to be a dynamically sized vector
    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    //ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    /**************************/
    /* Configure work vectors */
    /**************************/
    ssSetNumDWork(S,1);
    ssSetDWorkDataType(S,0,SS_POINTER);
    ssSetDWorkWidth(S,0,1);

    // see MATLABROOT/toolbox/simulink/simdemos/simfeatures/src/sfun_counter_cpp.cpp for an example
    ssSetNumPWork(S,1);

    //ssSetNumDWork(         S, 1);   /* number of DWork Vectors (persistent memory) */
    //ssSetNumRWork(         S, 0);   /* number of real work vector elements   */
    //ssSetNumIWork(         S, 0);   /* number of integer work vector elements*/
    //ssSetNumPWork(         S, 1);   /* number of pointer work vector elements*/
    //ssSetNumModes(         S, 0);   /* number of mode work vector elements   */
    //ssSetNumNonsampledZCs( S, 0);   /* number of nonsampled zero crossings   */
  /**
   * Definition of the Work Vectors:
   * Dwork0 -> 256 doubles - stores mavlink payload
   * Pwork -> 3 pointers - access to serial threads
   */
    // ssSetDworkWidth(S, 0, 256);
    // ssSetDworkDataType(S, 0, SS_DOUBLE);
    // ssSetDworkName(S, 0, "payload_data");

    
}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    mexLock();
    char port[] = "/dev/ttyUSB0";
    mavros::MavRos* pMATLink = new mavros::MavRos();
    ssGetPWork(S)[0] = (void *) pMATLink;
    pMATLink->init(port);
    pMATLink->hil_controls_.throttle = 1.23;
}
#endif /*  MDL_START */

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    /**************/
    /* Get MATLINK out of DWork */
    /**************/
    mavros::MavRos* pMATLink = (mavros::MavRos*) ssGetPWork(S)[0];

    /**************/
    /* Grab Input */
    /**************/

    mavlink_hil_vehicle_state_t vehicle_state;
    mavlink_hil_controls_t controls;

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;
    int_T message_type = (int_T) *uPtrs[0];
    //vehicle_state.time_usec = (uint64_t)*uPtrs[1];
    //vehicle_state.position[0] = *uPtrs[2];
    //vehicle_state.position[1] = *uPtrs[3];
    //vehicle_state.position[2] = *uPtrs[4];
    //vehicle_state.Va =          *uPtrs[5];
    //vehicle_state.alpha =       *uPtrs[6];
    //vehicle_state.beta  =       *uPtrs[7];
    //vehicle_state.phi =         *uPtrs[8];
    //vehicle_state.theta =       *uPtrs[9];
    //vehicle_state.psi =         *uPtrs[10];
    //vehicle_state.chi =         *uPtrs[11];
    //vehicle_state.p =           *uPtrs[12];
    //vehicle_state.q =           *uPtrs[13];
    //vehicle_state.r =           *uPtrs[14];
    //vehicle_state.Vg =          *uPtrs[15]; 
    //vehicle_state.wn =          *uPtrs[16];
    //vehicle_state.we =          *uPtrs[17];
    //vehicle_state.quat[0]     = *uPtrs[18];
    //vehicle_state.quat[1]     = *uPtrs[19];
    //vehicle_state.quat[2]     = *uPtrs[20];
    //vehicle_state.quat[3]     = *uPtrs[21];

    vehicle_state.time_usec = 1;
    vehicle_state.position[0] = 1.0;
    vehicle_state.position[1] =1.0;
    vehicle_state.position[2] =1.0;
    vehicle_state.Va =         1.0;
    vehicle_state.alpha =      1.0;
    vehicle_state.beta  =      1.0;
    vehicle_state.phi =        1.0;
    vehicle_state.theta =      1.0;
    vehicle_state.psi =         1.0;
    vehicle_state.chi =         1.0;
    vehicle_state.p =           1.0;
    vehicle_state.q =           1.0;
    vehicle_state.r =           1.0;
    vehicle_state.Vg =          1.0;
    vehicle_state.wn =          1.0;
    vehicle_state.we =          1.0;
    vehicle_state.quat[0]     = 0.0;
    vehicle_state.quat[1]     = 0.0;
    vehicle_state.quat[2]     = 0.0;
    vehicle_state.quat[3]     = 1.0;

    pMATLink->spinOnce(vehicle_state); // this line crashes simulink

    /************************/
    /* Receive from MAVLINK */
    /************************/

    // check if a new message has arrived
    controls.time_usec = pMATLink->hil_controls_.time_usec;
    controls.roll_ailerons = pMATLink->hil_controls_.roll_ailerons;
    controls.pitch_elevator = pMATLink->hil_controls_.pitch_elevator;
    controls.yaw_rudder = pMATLink->hil_controls_.yaw_rudder;
    controls.throttle = pMATLink->hil_controls_.throttle;
    controls.aux1 = pMATLink->hil_controls_.aux1;
    controls.aux2 = pMATLink->hil_controls_.aux2;
    controls.aux3 = pMATLink->hil_controls_.aux3;
    controls.aux4 = pMATLink->hil_controls_.aux4;
    controls.mode = pMATLink->hil_controls_.mode;
    controls.nav_mode = pMATLink->hil_controls_.nav_mode;

    /********************************************/
    /* Pack Received message into Output Vector */
    /********************************************/
    real_T *out = ssGetOutputPortRealSignal(S,0);
    out[0] = 0; // message type
    out[1] = controls.time_usec;
    out[2] = controls.roll_ailerons;
    out[3] = controls.pitch_elevator;
    out[4] = controls.yaw_rudder;
    out[5] = controls.throttle;
    out[6] = controls.aux1;
    out[7] = controls.aux2;
    out[8] = controls.aux3;
    out[9] = controls.aux4;
    out[10] = controls.mode;
    out[11] = controls.nav_mode;
}


static void mdlTerminate(SimStruct *S)
{
    mavros::MavRos* pMATLink = (mavros::MavRos*) ssGetPWork(S)[0];
    delete pMATLink;
    mexUnlock();
}





/* Required S-function trailer */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"    
#else
#include "cg_sfun.h"     
#endif

/* eof: sdotproduct.c */
