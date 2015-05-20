#define S_FUNCTION_NAME timestwo /* Defines and Includes */
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
static void mdlInitializeSizes(SimStruct *S)
{
    // sets the number of parameters that the S-Function Has.
    // For now, we don't have any, but in the future, we might need some.
    // Look at the the template file for guidance on how to deal with parameters
    // passed to the function
    ssSetNumSFcnParams(S, 0);


    // check to make sure that the right number of parameters were passed
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }



    /***************************************************/
    /* Take Care of the Continuous and Discrete States */
    /***************************************************/
    ssSetNumContStates(    S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(    S, 0);   /* number of discrete states             */




    /*****************************/
    /* Configure the input ports */
    /*****************************/
    if (!ssSetNumInputPorts(S, 1)) // If there is some number other than 1 input port to the Block
    { 
        return; // don't do anything
    }
    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED); // set the 0-th input port to a dynamically sized vector
    ssSetInputPortDirectFeedThrough(S, 0, 1); // directly feedthrough the data on port 1, because I'm going to use it immediately in mdlOutputs




    /******************************/
    /* Configure the output ports */
    /******************************/
    if (!ssSetNumOutputPorts(S,1)) // make sure there is only one output attached to the block
    {
        return;
    }
    ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED); // set the output to be a dynamically sized vector

    ssSetNumSampleTimes(S, 1); // I'm not exactly sure what this does.  We may want to use PORT_BASED_SAMPLE_TIMES to set port-based sample times

    /* Take care when specifying exception free code - see sfuntmpl.doc */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); // again, not sure what this does... we might not want to do this.


    /**************************/
    /* Configure work vectors */
    /**************************/
    ssSetNumDWork(         S, 1);   /* number of DWork Vectors (persistent memory) */
    ssSetNumRWork(         S, 0);   /* number of real work vector elements   */
    ssSetNumIWork(         S, 0);   /* number of integer work vector elements*/
    ssSetNumPWork(         S, 1);   /* number of pointer work vector elements*/
    ssSetNumModes(         S, 0);   /* number of mode work vector elements   */
    ssSetNumNonsampledZCs( S, 0);   /* number of nonsampled zero crossings   */
 
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
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME); // I'm not exactly sure what this does
    ssSetOffsetTime(S, 0, 0.0); // no offset on the sampling
}
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T i;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port
    real_T *y = ssGetOutputPortRealSignal(S,0); // returns a pointer to the output vector. (put output data here)
    int_T width = ssGetOutputPortWidth(S,0); // why is he checking the outputportwidth?

    for (i=0; i<width; i++) {
        *y++ = 2.0 *(*uPtrs[i]); // multiply all the items in the input port and put them into the output port
    }
}

static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif