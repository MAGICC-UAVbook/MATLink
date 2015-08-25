#include <stdio.h>
#include <cstdlib>

#include <mavros/mavconn_mavlink.h>
//#include "mavlinkClass.h"
#include "mavros/mavros.h"
#define num_waypoints 4

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
    fprintf(stderr, "Usage: %s serial_port rate\n\tserial_port: MAVLink serial port (e.g. /dev/ttyUSB0)\n\trate: loop rate in Hz\n", argv[0]);
    return 1;
    }

    int hz = atoi(argv[2]);
    int sleep = 1e6 / hz;

    // message to send waypoint data
    mavlink_new_waypoint_t new_waypoint;

    float Va = 9.5;//11;
//    float wps[5*num_waypoints] = {
//            0, 0, -100, 0, Va,
//            300, 0, -100, 45*M_PI/180, Va,
//            0, 300, -100, 45*M_PI/180, Va,
//            300, 300, -100, -135*M_PI/180, Va,
//           };
    float wps[5*num_waypoints] = {
            0, 0, -100, 0, Va,
            -175, 0, -100, -135*M_PI/180, Va,
            0, -175, -100, -135*M_PI/180, Va,
            -175, -175, -100, 45*M_PI/180, Va,
           };

    mavros::MavRos mavros(argv[1]);

    for (int i=0;i<num_waypoints;i++)
    {
        new_waypoint.w[0] = wps[i*5 + 0];
        new_waypoint.w[1] = wps[i*5 + 1];
        new_waypoint.w[2] = wps[i*5 + 2];
        new_waypoint.chi_d = wps[i*5 + 3];
        if(fabs(wps[i*5 + 3]) < 2*M_PI)
            new_waypoint.chi_valid = true;
        else
            new_waypoint.chi_valid = false;
        new_waypoint.Va_d = wps[i*5 + 4];


        mavros.spinOnce(new_waypoint);
        std::cout << "sent waypoint " << i << std::endl;

        usleep(sleep);
    }

  return 0;
}
