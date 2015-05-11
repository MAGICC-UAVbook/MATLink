/*=================================================================
 *
 * 
 *
 * The calling syntax is: TCPIP_Send_mex(ip_address, port_num, type, byte_order, data_in);
 *
 *		
 *
 *=================================================================*/

#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <time.h>
//#include <cstdint>

#include "mavlinkClass.h"

/* Input Arguments */

#define MESSAGE_TYPE	prhs[0]
#define SEQUENCE_NUMBER	prhs[1]
#define SYSTEM_ID		prhs[2]
#define COMPONENT_ID	prhs[3]
#define PAYLOAD			prhs[4]
#define COMM_PORT		prhs[5]

// Output Arguments
#define MESSAGE_TYPE_OUT	plhs[0]
#define SEQUENCE_NUMBER_OUT	plhs[1]
#define SYSTEM_ID_OUT		plhs[2]
#define COMPONENT_ID_OUT	plhs[3]
#define PAYLOAD_OUT			plhs[4]

typedef unsigned char uint8_t;

struct MessageHeader
{
	uint8_t messageType;
	uint8_t sequenceNumber;
	uint8_t systemID;
	uint8_t componentID;
};

struct Params
{
	short comm_port; // What data type?
	short something_else;
};


bool init = false;
//bool m_connected = false;

//unsigned char data_recv[RECV_BUFFER_SIZE];
int data_length;

double curr_time = 0, last_time = 0;


// mex_exit_function is called when matlab exits.  
// It's used to free up memory and close sockets
static void mex_exit_function()
{
	mexUnlock(); // Unlock the mex file so matlab can remove it from memory
	//disconnect(); // close the sockets
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{ 
    unsigned char *data_in; 
	char *type, *byte_order;
	Params *parameters = new Params;
	MessageHeader *messageHeader = new MessageHeader;
    mwSize m_data,n_data; 
    int length, bytes;
	double *pointer;
	unsigned char *payload;

	double m_size, n_size;
    
    /* Check for proper number of arguments */
    
    if (nrhs != 6) 
		mexErrMsgTxt("MAVLink_Mex: 6 input argument required."); 

    /* Check the types */
	//if (!mxIsChar(IP_ADDRESS))
	//	mexErrMsgTxt("TCPIP_Send: IP_ADDRESS requires a string.");
   
	/* Check the dimensions */
	m_data = mxGetM(MESSAGE_TYPE);
	n_data = mxGetN(MESSAGE_TYPE);
    if (mxIsComplex(MESSAGE_TYPE) || m_data != 1 || n_data != 1 )  
		mexErrMsgTxt("MAVLink_Mex: MESSAGE_TYPE requires a single real number");

	m_data = mxGetM(SEQUENCE_NUMBER);
	n_data = mxGetN(SEQUENCE_NUMBER);
    if (mxIsComplex(SEQUENCE_NUMBER) || m_data != 1 || n_data != 1 )  
		mexErrMsgTxt("MAVLink_Mex: SEQUENCE_NUMBER requires a single real number");

	m_data = mxGetM(SYSTEM_ID);
	n_data = mxGetN(SYSTEM_ID);
    if (mxIsComplex(SYSTEM_ID) || m_data != 1 || n_data != 1 )  
		mexErrMsgTxt("MAVLink_Mex: SYSTEM_ID requires a single real number");

	m_data = mxGetM(COMPONENT_ID);
	n_data = mxGetN(COMPONENT_ID);
    if (mxIsComplex(COMPONENT_ID) || m_data != 1 || n_data != 1 )  
		mexErrMsgTxt("MAVLink_Mex: COMPONENT_ID requires a single real number");

    m_data = mxGetM(PAYLOAD); 
    n_data = mxGetN(PAYLOAD);
    if (mxIsComplex(PAYLOAD) || (m_data == 0) || (n_data == 0))
		mexErrMsgTxt("MAVLink_Mex: PAYLOAD requires a M x N real vector.");

	/* Get the inputs */
	pointer = mxGetPr(MESSAGE_TYPE);
	messageHeader->messageType = (uint8_t)(*pointer);

	pointer = mxGetPr(SEQUENCE_NUMBER);
	messageHeader->sequenceNumber = (uint8_t)(*pointer);

	pointer = mxGetPr(SYSTEM_ID);
	messageHeader->systemID = (uint8_t)(*pointer);

	pointer = mxGetPr(COMPONENT_ID);
	messageHeader->componentID = (uint8_t)(*pointer);

	payload = (unsigned char *)mxGetData(PAYLOAD);

	// Init
	if(!init)
	{
		mexLock(); // Lock the mex function to prevent variables from being removed from memory
        mexAtExit(mex_exit_function); // Set the function to be called when MATLAB exits (to free memory and close sockets)
		init = true;
	}

  mavlinkClass mavlink("/dev/ttyUSB0");

  // Format and send the packet recieved from MATLAB to the PixHawk
  mavlink_hil_vehicle_state_t vehicle_state;

  vehicle_state.usec = 0.0f; // not sure what to do here
  vehicle_state.position[0] = payload[0];
  vehicle_state.position[1] = payload[1];
  vehicle_state.position[2] = payload[2];
  vehicle_state.Va    = payload[3];
  vehicle_state.alpha = payload[4];
  vehicle_state.beta  = payload[5];
  vehicle_state.phi   = payload[6];
  vehicle_state.theta = payload[7];
  vehicle_state.psi   = payload[8];
  vehicle_state.chi   = payload[9];
  vehicle_state.p  = payload[10];
  vehicle_state.q  = payload[11];
  vehicle_state.r  = payload[12];
  vehicle_state.Vg = payload[13];
  vehicle_state.wn = payload[14];
  vehicle_state.we = payload[15];
  vehicle_state.quat[0] = payload[16];
  vehicle_state.quat[1] = payload[17];
  vehicle_state.quat[2] = payload[18];
  vehicle_state.quat[3] = payload[19];
  vehicle_state.quat_valid = (uint8_t)payload[20]; // this may give weird behavior

  mavlink.send_vehicle_state(vehicle_state);

  // Receive packet and format the data


	// create output data structures
	MESSAGE_TYPE_OUT = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
	SEQUENCE_NUMBER_OUT = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
	SYSTEM_ID_OUT = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
	COMPONENT_ID_OUT = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
	PAYLOAD_OUT = mxCreateNumericMatrix(256, 1, mxDOUBLE_CLASS, mxREAL); // max possible number of payload parameters

	// populate data structures


	delete messageHeader;
    mexUnlock();
}
