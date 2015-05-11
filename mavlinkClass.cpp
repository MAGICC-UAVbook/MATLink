#include <stdio.h>
#include "mavlinkClass.h"

mavlinkClass::mavlinkClass(char* serialName){
  fd = open(serialName, O_RDWR | O_NOCTTY | O_NDELAY);
//check that we opened the port
	if (fd == -1){
		perror("Error opening port");
		classStatus = 0;
		return;
	}
	else{
		// set read() calls to be nonblocking
		fcntl(fd, F_SETFL, FNDELAY);
		classStatus = 1;
	}
	send_counter = 0;
}


bool mavlinkClass::getClassStatus(){
	return classStatus;
}

void mavlinkClass::handle_mavlink_msg(){
	switch(msg.msgid){
		case MAVLINK_MSG_ID_HEARTBEAT:
			handle_heartbeat();
			break;
		default:
      fprintf(stderr, "Received unhandled MAVlink message, ID: [%i]", msg.msgid);
			break;
	}
}

void mavlinkClass::handle_heartbeat(){
  printf("Received MAVlink heartbeat message, ID: [%i]", msg.msgid);
}

void mavlinkClass::handle_hil_controls()
{
}

void mavlinkClass::send_vehicle_state(const mavlink_hil_vehicle_state_t &vehicle_state)
{
  // send garbage
  uint8_t tt = 254;
  write(fd, &tt, 1);

  // pack message
  mavlink_msg_hil_vehicle_state_encode(42, 0, &msg, &vehicle_state);

  // copy message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buffy, &msg);

  // write message to serial
  if (write(fd, buffy, len) < 0)
  {
    fprintf(stderr, "Sending vehicle state failed\n");
  }
  else
  {
    printf("Sent vehicle state to UART, message id: [%i]", msg.msgid);
  }
}

void mavlinkClass::check_serial(){
// the following may need to be changed to a while() loop - I left as an if() 
// 	because I didn't want to get stuck if the receive buffer fills too fast
	if(read(fd,&charval,1) > 0){
		//if we receive character, try to parse it
		if (mavlink_parse_char(MAVLINK_COMM_0, charval, &msg, &status)){
			handle_mavlink_msg();
		}
	}
}

int mavlinkClass::flush_buffer(){
	return tcflush(fd,TCIOFLUSH);
}

int mavlinkClass::flush_buffer(int queue){
	return tcflush(fd,queue);
}
