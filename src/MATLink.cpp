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
#define M_RAD_TO_DEG_F 	57.2957795130823f
#define EARTH_RADIUS 6378145.0f
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
    ssSetInputPortWidth(S, 0, 14);
    ssSetInputPortDirectFeedThrough(S, 0, 1); // we will use the input in the output step

    /******************************/
    /* Configure the output ports */
    /******************************/
    if (!ssSetNumOutputPorts(S, 1)) return;
    if(!ssSetOutputPortVectorDimension(S, 0, 1)) return;
    ssSetOutputPortWidth(S, 0, 16); // set the output to be a dynamically sized vector
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
    char port[] = "/dev/ttyUSB1";
    mavros::MavRos* pMATLink = new mavros::MavRos(port);
    ssGetPWork(S)[0] = (void *) pMATLink;
}
#endif /*  MDL_START */

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.01);//INHERITED_SAMPLE_TIME);//CONTINUOUS_SAMPLE_TIME);//mxGetScalar(ssGetSFcnParam(S, 0)));
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

    mavlink_hil_sensor_t sensor;
    mavlink_hil_gps_t gps;
    mavlink_hil_vehicle_state_t state;

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;
    int_T message_type = (int_T) *uPtrs[0];
    sensor.time_usec = (uint64_t)*uPtrs[19]/1000000;
    sensor.xgyro              = *uPtrs[0];
    sensor.ygyro              = *uPtrs[1];
    sensor.zgyro              = *uPtrs[2];
    sensor.xacc               = *uPtrs[3];
    sensor.yacc               = *uPtrs[4];
    sensor.zacc               = *uPtrs[5];
    sensor.abs_pressure       = (*uPtrs[6])/100; // 1 mbar == 100 pa
    sensor.diff_pressure      = (*uPtrs[7])/100; // 1 mbar == 100 pa
    float gps_n               = *uPtrs[8];
    float gps_e               = *uPtrs[9];
    float gps_h               = *uPtrs[10];
    float gps_Vg              = *uPtrs[11];
    float gps_course          = *uPtrs[12];

//    if(gps_n != pMATLink->gps_n_old || gps_e != pMATLink->gps_e_old || gps_Vg != pMATLink->gps_Vg_old)// || gps_course != pMATLink->gps_course_old)
//    {
//        float conversion = (3.14159/(180*1e7)); //from 1/10th of a micro degree to radian
//        gps.lat = gps_n / EARTH_RADIUS / conversion;
//        gps.lon = gps_e / EARTH_RADIUS / conversion;
//        gps.alt = gps_h / 1e-3f;
//        gps.vel = gps_Vg / 1e-2f;
//        gps.vn = 0;
//        gps.ve = 0;
//        gps.vd = 0;
//        if(gps_course < 0)
//            gps_course += 2*3.14159;
//        gps.cog = gps_course * M_RAD_TO_DEG_F / 1e-2f;
//        std::cout << "gps course: " << gps_course << std::endl;
//        gps.fix_type = 3;

//        pMATLink->gps_n_old = gps_n;
//        pMATLink->gps_e_old = gps_e;
//        pMATLink->gps_Vg_old = gps_Vg;
//        pMATLink->gps_course_old = gps_course;

//        pMATLink->spinOnce(sensor, gps, true);
//    }
//    else
//        pMATLink->spinOnce(sensor, gps, false);

    /************************/
    /* Receive from MAVLINK */
    /************************/

    // check if a new message has arrived
    state.time_usec = pMATLink->hil_vehicle_state_.time_usec;
    state.position[0] = pMATLink->hil_vehicle_state_.position[0];
    state.position[1] = pMATLink->hil_vehicle_state_.position[1];
    state.position[2] = pMATLink->hil_vehicle_state_.position[2];
    state.Va = pMATLink->hil_vehicle_state_.Va;
    state.alpha = pMATLink->hil_vehicle_state_.alpha;
    state.beta = pMATLink->hil_vehicle_state_.beta;
    state.phi = pMATLink->hil_vehicle_state_.phi;
    state.theta = pMATLink->hil_vehicle_state_.theta;
    state.psi = pMATLink->hil_vehicle_state_.psi;
    state.chi = pMATLink->hil_vehicle_state_.chi;
    state.p = pMATLink->hil_vehicle_state_.p;
    state.q = pMATLink->hil_vehicle_state_.q;
    state.r = pMATLink->hil_vehicle_state_.r;
    state.Vg = pMATLink->hil_vehicle_state_.Vg;
    state.wn = pMATLink->hil_vehicle_state_.wn;
    state.we = pMATLink->hil_vehicle_state_.we;

    if(pMATLink->msg_received = true) {
        pMATLink->msg_received = false;
//        std::cout << pMATLink->hil_controls_.throttle << " " << vehicle_state.position[2] << " "  << vehicle_state.q << std::endl;
    }
    /********************************************/
    /* Pack Received message into Output Vector */
    /********************************************/
    real_T *out = ssGetOutputPortRealSignal(S,0);
    out[0] = state.position[0];
    out[1] = state.position[1];
    out[2] = state.position[2];
    out[3] = state.Va;
    out[4] = state.alpha;
    out[5] = state.beta;
    out[6] = state.phi;
    out[7] = state.theta;
    out[8] = state.chi;
    out[9] = state.p;
    out[10] = state.q;
    out[11] = state.r;
    out[12] = state.Vg;
    out[13] = state.wn;
    out[14] = state.we;
    out[15] = state.psi;
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
