#include <stdio.h>
#include "mavlinkClass.h"

mavlinkClass::mavlinkClass(char* serialName){
  fd_ = open(serialName, O_RDWR | O_NOCTTY | O_NDELAY);
//check that we opened the port
  if (fd_ == -1){
		perror("Error opening port");
    classStatus_ = 0;
		return;
	}
	else{
		// set read() calls to be nonblocking
    fcntl(fd_, F_SETFL, FNDELAY);
    classStatus_ = 1;
	}
  send_counter_ = 0;
}


bool mavlinkClass::getClassStatus(){
  return classStatus_;
}

void mavlinkClass::send_vehicle_state(const mavlink_hil_vehicle_state_t &vehicle_state)
{
  // send garbage
  uint8_t tt = 254;
  write(fd_, &tt, 1);

  // pack message
  mavlink_msg_hil_vehicle_state_encode(42, 0, &msg_, &vehicle_state);

  // copy message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buffy_, &msg_);

  // write message to serial
  if (write(fd_, buffy_, len) < 0)
  {
    fprintf(stderr, "Sending vehicle state failed\n");
  }
  else
  {
    printf("Sent vehicle state to UART, message id: [%i]", msg_.msgid);
  }
}

bool mavlinkClass::check_serial(mavlink_message_t *msg){
// the following may need to be changed to a while() loop - I left as an if() 
// 	because I didn't want to get stuck if the receive buffer fills too fast
  if(read(fd_,&charval_,1) > 0){
		//if we receive character, try to parse it
    if (mavlink_parse_char(MAVLINK_COMM_0, charval_, &msg_, &status_)){

      if (msg_.msgid == MAVLINK_MSG_ID_HEARTBEAT) // handle heartbeat messages here
      {
        printf("Received MAVlink heartbeat message, ID: [%i]", msg_.msgid);
        return false;
      }
      else // pass out all other message types
      {
        memcpy(msg, &msg_, sizeof(&msg_)); // copy message data to output variable
        return true;
      }
		}
	}
}

int mavlinkClass::flush_buffer(){
  return tcflush(fd_,TCIOFLUSH);
}

int mavlinkClass::flush_buffer(int queue){
  return tcflush(fd_,queue);
}
