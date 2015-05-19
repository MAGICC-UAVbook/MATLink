#include <stdio.h>
#include <cstdlib>

#include "include/mavlink/v1.0/MAGICC/mavlink.h"
#include "mavlinkClass.h"

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    fprintf(stderr, "Usage: %s serial_port rate\n\tserial_port: MAVLink serial port (e.g. /dev/ttyUSB0)\n\trate: loop rate in Hz\n", argv[0]);
    return 1;
  }

  int hz = atoi(argv[2]);
  int sleep = 1e6 / hz;

  // message to send (garbage data)
  mavlink_hil_vehicle_state_t vehicle_state;
  vehicle_state.position[1] = 3.5;
  vehicle_state.q = 4.2;

  mavlinkClass mavlink(argv[1]);

  for (;;)
  {
    vehicle_state.q += 0.1;
    mavlink.send_vehicle_state(vehicle_state);

    mavlink_message_t msg;
    mavlink.check_serial(&msg);
    printf("\n");

    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_HIL_CONTROLS:
      mavlink_hil_controls_t controls;
      mavlink_msg_hil_controls_decode(&msg, &controls);
      printf("Received HIL_CONTROLS message:\n\troll: %f\n\tpitch: %f\n\tyaw: %f\n\tthrottle: %f\n",
             controls.roll_ailerons,
             controls.pitch_elevator,
             controls.yaw_rudder,
             controls.throttle);
      break;
    case 0:
      // do nothing
      break;
    default:
      printf("Received unhandled mavlink message, id=%i\n", msg.msgid);
      break;
    }

    usleep(sleep);
  }

  return 0;
}
