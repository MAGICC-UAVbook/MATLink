#include <stdio.h>
#include <cstdlib>

#include <mavros/mavconn_mavlink.h>
//#include "mavlinkClass.h"
#include "mavros/mavros.h"

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
  vehicle_state.q = 0.2;
  mavlink_hil_controller_commands_t controller_commands;
  controller_commands.h_c = 5;

  mavros::MavRos mavros(argv[1]);
  mavlink_hil_controls_t controls;

  for (;;)
  {
    vehicle_state.q += 0.1;
    if(vehicle_state.q > 1)
        vehicle_state.q = 0;
    controller_commands.h_c++;
    if(controller_commands.h_c >= 500)
        controller_commands.h_c = 5;
//    mavlink.send_vehicle_state(vehicle_state);
    mavros.spinOnce(vehicle_state, controller_commands);
    std::cout << ": " << mavros.hil_controls_.roll_ailerons
              << ", " << mavros.hil_controls_.pitch_elevator
              << ", " << mavros.hil_controls_.yaw_rudder
              << ", " << mavros.hil_controls_.throttle
              << std::endl;

//    mavlink_message_t msg;
//    mavlink.check_serial(&msg);
//    printf("\n");
    usleep(sleep);
  }

  return 0;
}
