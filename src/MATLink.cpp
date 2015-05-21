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

#include "simstruc.h"

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

    /*******************************/
    /* SET UP MAVLINK COMMUNICTION */
    /*******************************/
    // define serial port
    // create vehicle state message
    // initialize data communication
    // save pointers to the PWork Vector
}


static void mdlStart(SimStruct *S)
{
    real_T *payload_data = (real_T*) ssGetDWork(S,0);
    for(int i = 1; i<ssGetDWorkWidth(S,0);i++)
    {
        payload_data[i] = 0.0; // set it all to zeros to avoid weird returns
    }
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    /*
     * Structure of Input
     * MESSAGE_ID
     */

     /**************/
     /* Grab Input */
     /**************/
     InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port
     int_T message_type = (int_T) *uPtrs[0];
     real_T t  =          *uPtrs[1];
     real_T pn =          *uPtrs[2];
     real_T pe =          *uPtrs[3];
     real_T h =           *uPtrs[4];
     real_T u =           *uPtrs[5];
     real_T v =           *uPtrs[6];
     real_T w =           *uPtrs[7];
     real_T alpha =       *uPtrs[8];
     real_T beta  =       *uPtrs[9];
     real_T phi =         *uPtrs[10];
     real_T theta =       *uPtrs[11];
     real_T psi =         *uPtrs[12];
     real_T p =           *uPtrs[13];
     real_T q =           *uPtrs[14];
     real_T r =           *uPtrs[15];
     real_T gamma =       *uPtrs[16];
     real_T chi =         *uPtrs[17];
     real_T wn =          *uPtrs[18];
     real_T we =          *uPtrs[19];
     real_T wd =          *uPtrs[20];

     /*******************/
     /* Send on MAVLINK */
     /*******************/

     /************************/
     /* Receive from MAVLINK */
     /************************/

     /********************************************/
     /* Pack Received message into Output Vector */
     /********************************************/
     int_T message_output_type = 1;
     real_T output_time = 0.0;
     real_T roll_ailerons = 5.0;
     real_T pitch_elevator = 15.0;
     real_T yaw_rudder = 500.0;
     real_T throttle = 1000.0;

     real_T *out = ssGetOutputPortRealSignal(S,0);
     out[0] = (real_T) message_output_type;
     out[1] = output_time;
     out[2] = roll_ailerons;
     out[3] = pitch_elevator;
     out[4] = yaw_rudder;
     out[5] = throttle;
}


static void mdlTerminate(SimStruct *S)
{
}


/* Required S-function trailer */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"    
#else
#include "cg_sfun.h"     
#endif

/* eof: sdotproduct.c */