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
//#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
//!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

//#define MDL_CHECK_PARAMETERS
//#if defined(MDL_CHECK_PARAMETERS)
///*
// * Check to make sure that each parameter is 1-d and positive
// */
//static void mdlCheckParameters(SimStruct *S)
//{

//    const mxArray *pVal0 = ssGetSFcnParam(S,0);

//    if ( !IS_PARAM_DOUBLE(pVal0)) {
//        ssSetErrorStatus(S, "Parameter to S-function must be a double scalar");
//        return;
//    }
//}
//#endif

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
    } /*else {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    }*/

    /***************************************************/
    /* Take Care of the Continuous and Discrete States */
    /***************************************************/
    ssSetNumContStates(S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(S, 0);   /* number of discrete states             */



    /*****************************/
    /* Configure the input ports */
    /*****************************/
    if(!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 20);
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
    mavros::MavRos* pMATLink = new mavros::MavRos(port);
    ssGetPWork(S)[0] = (void *) pMATLink;
}
#endif /*  MDL_START */

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);//CONTINUOUS_SAMPLE_TIME);//mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    /**************/
    /* Get MATLINK out of PWork */
    /**************/
    mavros::MavRos* pMATLink = (mavros::MavRos*) ssGetPWork(S)[0];

    /**************/
    /* Grab Input */
    /**************/

    mavlink_hil_vehicle_state_t vehicle_state;
    mavlink_hil_controller_commands_t controller_commands;
    mavlink_hil_controls_t controls;

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;
    int_T message_type = (int_T) *uPtrs[0];
    vehicle_state.time_usec = (uint64_t)*uPtrs[19]/1000000;
    vehicle_state.position[0] = *uPtrs[0];
    vehicle_state.position[1] = *uPtrs[1];
    vehicle_state.position[2] = *uPtrs[2];
    vehicle_state.Va          = *uPtrs[3];
    vehicle_state.alpha       = *uPtrs[4];
    vehicle_state.beta        = *uPtrs[5];
    vehicle_state.phi         = *uPtrs[6];
    vehicle_state.theta       = *uPtrs[7];
    vehicle_state.psi         = *uPtrs[15];
    vehicle_state.chi         = *uPtrs[8];
    vehicle_state.p           = *uPtrs[9];
    vehicle_state.q           = *uPtrs[10];
    vehicle_state.r           = *uPtrs[11];
    vehicle_state.Vg          = *uPtrs[12];
    vehicle_state.wn          = *uPtrs[13];
    vehicle_state.we          = *uPtrs[14];
    vehicle_state.quat[0]     = 0;
    vehicle_state.quat[1]     = 0;
    vehicle_state.quat[2]     = 0;
    vehicle_state.quat[3]     = 0;

    controller_commands.Va_c  = *uPtrs[16];
    controller_commands.h_c   = *uPtrs[17];
    controller_commands.chi_c = *uPtrs[18];

    pMATLink->spinOnce(vehicle_state, controller_commands);

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

    if(pMATLink->msg_received = true) {
        pMATLink->msg_received = false;
        std::cout << pMATLink->hil_controls_.throttle << " " << vehicle_state.position[2] << " "  << vehicle_state.q << std::endl;
    }
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
    out[8] = controller_commands.Va_c;//controls.aux3;
    out[9] = controller_commands.h_c;//controls.aux4;
    out[10] = controller_commands.chi_c;//controls.mode;
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
