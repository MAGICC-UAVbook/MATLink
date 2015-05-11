#ifndef __MAVLINK_CLASS_H__
#define __MAVLINK_CLASS_H__

/*
mavlinkClass.h

mavlinkClass object for sending MAVLINK messages

TDW
twoodbury@tamu.edu
*/

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "include/mavlink/v1.0/MAGICC/mavlink.h"

class mavlinkClass{
	public:
    mavlinkClass(char* serialName);
		void handle_mavlink_msg();
    void handle_heartbeat();
    void handle_hil_controls();
    void send_vehicle_state(const mavlink_hil_vehicle_state_t &vehicle_state);

		bool getClassStatus();
    void check_serial(); //!< check serial port for new messages
    int flush_buffer(); //!< flush input and output by default
    int flush_buffer(int queue); //!< specify input/output buffer
	private:
    int fd; //!< serial terminal integer reference
    uint8_t buffy[MAVLINK_MAX_PACKET_LEN]; //!< buffer that we pack serial messages into
    uint8_t charval; //!< we read serial characters into here when we need to
		mavlink_message_t msg;
		mavlink_status_t status;
    bool classStatus; //!< set to 0 if we can't open the port, 1 otherwise
    unsigned long send_counter; //!< use this value to count how many messages we have sent and periodically flush the buffer
};

#endif
