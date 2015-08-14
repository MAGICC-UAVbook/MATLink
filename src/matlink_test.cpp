#include <stdio.h>
#include <cstdlib>

#include <mavros/mavconn_mavlink.h>
//#include "mavlinkClass.h"
#include "mavros/mavros.h"

int main(int argc, char* argv[])
{
  if (argc < 4)
  {
    fprintf(stderr, "Usage: %s serial_port rate state\n\tserial_port: MAVLink serial port (e.g. /dev/ttyUSB0)\n\trate: loop rate in Hz\n\tstate: what you want to print\n", argv[0]);
    return 1;
  }

  int hz = atoi(argv[2]);
  int sleep = 1e6 / hz;

  int state = atoi(argv[3]);
  if (state > 15 || state < 0)
  {
    std::cout << "Not a valid state" << std::endl;
    return 1;
  }

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
    mavlink_hil_sensor_t sensor;
    mavlink_hil_gps_t gps;
    gps.lat = 0;
    gps.lon = 0;
    gps.alt = 0;
    //mavros.spinOnce(sensor, gps, false);
    std::cout << ": " << mavros.msg_received;
    switch(state)
    {
    case 0:
        std::cout << ", n position " << mavros.hil_vehicle_state_.position[0] << std::endl;
        break;
    case 1:
        std::cout << ", e position " << mavros.hil_vehicle_state_.position[1] << std::endl;
        break;
    case 2:
        std::cout << ", height " << mavros.hil_vehicle_state_.position[2] << ", Va " << mavros.hil_vehicle_state_.Va << std::endl;
        break;
    case 3:
        std::cout << ", Va " << mavros.hil_vehicle_state_.Va << std::endl;
        break;
    case 4:
        std::cout << ", alpha " << mavros.hil_vehicle_state_.alpha*180/3.14159 << std::endl;
        break;
    case 5:
        std::cout << ", beta " << mavros.hil_vehicle_state_.beta*180/3.14159 << std::endl;
        break;
    case 6:
        std::cout << ", phi " << mavros.hil_vehicle_state_.phi*180/3.14159 << std::endl;
        break;
    case 7:
        std::cout << ", theta " << mavros.hil_vehicle_state_.theta*180/3.14159 << std::endl;
        break;
    case 8:
        std::cout << ", psi " << mavros.hil_vehicle_state_.psi*180/3.14159 << std::endl;
        break;
    case 9:
        std::cout << ", chi " << mavros.hil_vehicle_state_.chi*180/3.14159 << std::endl;
        break;
    case 10:
        std::cout << ", p " << mavros.hil_vehicle_state_.p << std::endl;
        break;
    case 11:
        std::cout << ", q " << mavros.hil_vehicle_state_.q << std::endl;
        break;
    case 12:
        std::cout << ", r " << mavros.hil_vehicle_state_.r << std::endl;
        break;
    case 13:
        std::cout << ", Vg " << mavros.hil_vehicle_state_.Vg << std::endl;
        break;
    case 14:
        std::cout << ", wn " << mavros.hil_vehicle_state_.wn << std::endl;
        break;
    case 15:
        std::cout << ", we " << mavros.hil_vehicle_state_.we << std::endl;
        break;

    }
//    std::cout << ": " << mavros.msg_received
//              << ", " << mavros.hil_vehicle_state_.position[0]
//              << ", " << mavros.hil_vehicle_state_.position[1]
//              << ", " << mavros.hil_vehicle_state_.position[2]
//              << ", " << mavros.hil_vehicle_state_.Va
//              << ", " << mavros.hil_vehicle_state_.alpha
//              << ", " << mavros.hil_vehicle_state_.beta
//              << ", " << mavros.hil_vehicle_state_.phi
//              << ", " << mavros.hil_vehicle_state_.theta
//              << ", " << mavros.hil_vehicle_state_.psi
//              << ", " << mavros.hil_vehicle_state_.chi
//              << ", " << mavros.hil_vehicle_state_.p
//              << ", " << mavros.hil_vehicle_state_.q
//              << ", " << mavros.hil_vehicle_state_.r
//              << ", " << mavros.hil_vehicle_state_.Vg
//              << ", " << mavros.hil_vehicle_state_.wn
//              << ", " << mavros.hil_vehicle_state_.we
//              << std::endl;
    mavros.msg_received = false;

//    mavlink_message_t msg;
//    mavlink.check_serial(&msg);
//    printf("\n");
    usleep(sleep);
  }

  return 0;
}
