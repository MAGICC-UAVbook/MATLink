/**
 * @brief MAVROS class
 * @file mavros.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros.h>
#include <mavros/utils.h>
#include <fnmatch.h>
#include <iostream>

using namespace mavros;
using namespace mavconn;

MavRos::MavRos()
{}

void MavRos::init(char* serialName) //:
//	node_handle(nh_),
//	mavlink_node_handle("/mavlink"), // for compatible reasons
//	fcu_link_diag("FCU connection"),
//  gcs_link_diag("GCS bridge")//,
//	plugin_loader("mavros", "mavplugin::MavRosPlugin"),
//	message_route_table{}
{
    std::string fcu_url(serialName), gcs_url;
    int system_id = 1;
    int component_id = 1;
	int tgt_system_id, tgt_component_id;
    bool px4_usb_quirk = false;
	MAVConnInterface::Ptr fcu_link;

//	node_handle.param<std::string>("fcu_url", fcu_url, "serial:///dev/ttyACM0");
//	node_handle.param<std::string>("gcs_url", gcs_url, "udp://@");
//	node_handle.param("system_id", system_id, 1);
//	node_handle.param<int>("component_id", component_id, MAV_COMP_ID_UDP_BRIDGE);
//	node_handle.param("target_system_id", tgt_system_id, 1);
//	node_handle.param("target_component_id", tgt_component_id, 1);
//	node_handle.param("startup_px4_usb_quirk", px4_usb_quirk, false);
//	node_handle.getParam("plugin_blacklist", plugin_blacklist);

    //diag_updater.setHardwareID("Mavlink");

    //ROS_INFO_STREAM("FCU URL: " << fcu_url);
	try {
		fcu_link = MAVConnInterface::open_url(fcu_url, system_id, component_id);
		// may be overridden by URL
		system_id = fcu_link->get_system_id();
		component_id = fcu_link->get_component_id();

        //fcu_link_diag.set_mavconn(fcu_link);
        //diag_updater.add(fcu_link_diag);
	}
	catch (mavconn::DeviceError &ex) {
        //ROS_FATAL("FCU: %s", ex.what());
        //ros::shutdown();
		return;
	}

	if (gcs_url != "") {
        //ROS_INFO_STREAM("GCS URL: " << gcs_url);
		try {
			gcs_link = MAVConnInterface::open_url(gcs_url, system_id, component_id);

            //gcs_link_diag.set_mavconn(gcs_link);
            //diag_updater.add(gcs_link_diag);
		}
		catch (mavconn::DeviceError &ex) {
            //ROS_FATAL("GCS: %s", ex.what());
            //ros::shutdown();
			return;
		}
	}
	else
        std::cout << "GCS bridge disabled" << std::endl;

	// ROS mavlink bridge
    //mavlink_pub = mavlink_node_handle.advertise<Mavlink>("from", 100);
    //mavlink_sub = mavlink_node_handle.subscribe("to", 100, &MavRos::mavlink_sub_cb, this,
//			ros::TransportHints()
//			.unreliable()
//			.maxDatagramSize(1024));

    fcu_link->message_received.connect(boost::bind(&MavRos::mavlink_receive, this, _1, _2, _3));
//	fcu_link->message_received.connect(boost::bind(&MavRos::plugin_route_cb, this, _1, _2, _3));
	fcu_link->port_closed.connect(boost::bind(&MavRos::terminate_cb, this));

	if (gcs_link) {
		fcu_link->message_received.connect(
				boost::bind(&MAVConnInterface::send_message, gcs_link, _1, _2, _3));
		gcs_link->message_received.connect(
				boost::bind(&MAVConnInterface::send_message, fcu_link, _1, _2, _3));
        //gcs_link_diag.set_connection_status(true);
	}

	mav_uas.set_tgt(tgt_system_id, tgt_component_id);
	UAS_FCU(&mav_uas) = fcu_link;
    //mav_uas.sig_connection_changed.connect(boost::bind(&MavlinkDiag::set_connection_status, &fcu_link_diag, _1));
	mav_uas.sig_connection_changed.connect(boost::bind(&MavRos::log_connect_change, this, _1));

//	for (auto &name : plugin_loader.getDeclaredClasses())
//		add_plugin(name);

	if (px4_usb_quirk)
		startup_px4_usb_quirk();

    std::cout << "MAVROS started. MY ID [" <<  system_id << ", " << component_id << "], TARGET ID [" << tgt_system_id << ", "<< tgt_component_id << "]" <<  std::endl;

    // reset the state member hil_controls so that it doesn't output weird stuff
    hil_controls_.time_usec = 0;
    hil_controls_.roll_ailerons = 0;
    hil_controls_.pitch_elevator =  0;
    hil_controls_.yaw_rudder = 0;
    hil_controls_.throttle = 0;
    hil_controls_.aux1 = 0;
    hil_controls_.aux2 = 0;
    hil_controls_.aux3 = 0;
    hil_controls_.aux4 = 0;
    hil_controls_.mode = 0;
    hil_controls_.nav_mode = 0;
}

void MavRos::spinOnce(mavlink_hil_vehicle_state_t vs)
{

//	ros::Rate loop_rate(1000);
//	while (node_handle.ok()) {
//		ros::spinOnce();
//		diag_updater.update();

//		loop_rate.sleep();
//	}
    mavlink_message_t mmsg;
    mavlink_msg_hil_vehicle_state_encode(1,1,&mmsg,&vs);
    mavlink_send(mmsg);
    //diag_updater.update();

//	ROS_INFO("Stopping mavros...");
//	mav_uas.stop();
}

void MavRos::mavlink_receive(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
    //MavlinkPtr rmsg = boost::make_shared<Mavlink>();

//	if  (mavlink_pub.getNumSubscribers() == 0)
//		return;

    //rmsg->header.stamp = ros::Time::now();
    //mavutils::copy_mavlink_to_ros(mmsg, rmsg);
    //mavlink_pub.publish(rmsg);

    std::cout << "received message, id: " << (int)mmsg->msgid;
    if(mmsg->msgid == MAVLINK_MSG_ID_HIL_CONTROLS)
    {
        mavlink_msg_hil_controls_decode(mmsg, &hil_controls_);
    }
}

void MavRos::mavlink_send(mavlink_message_t mmsg) {

    UAS_FCU(&mav_uas)->send_message(&mmsg, 1, 1);
}

//void MavRos::plugin_route_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
//	message_route_table[mmsg->msgid](mmsg, sysid, compid);
//}

//bool MavRos::check_in_blacklist(std::string &pl_name) {
//	for (auto &pattern : plugin_blacklist) {
//		int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
//		if (cmp == 0)
//			return true;
//		else if (cmp != FNM_NOMATCH)
//			ROS_ERROR("Blacklist check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
//					pattern.c_str(), pl_name.c_str(), cmp);
//	}

//	return false;
//}

//void MavRos::add_plugin(std::string &pl_name) {
//	boost::shared_ptr<mavplugin::MavRosPlugin> plugin;

//	if (check_in_blacklist(pl_name)) {
//		ROS_INFO_STREAM("Plugin [alias " << pl_name << "] blacklisted");
//		return;
//	}

//	try {
//		plugin = plugin_loader.createInstance(pl_name);
//		plugin->initialize(mav_uas, node_handle, diag_updater);
//		loaded_plugins.push_back(plugin);
//		std::string repr_name = plugin->get_name();

//		ROS_INFO_STREAM("Plugin " << repr_name <<
//				" [alias " << pl_name << "] loaded and initialized");

//		for (auto &pair : plugin->get_rx_handlers()) {
//			ROS_DEBUG("Route msgid %d to %s", pair.first, repr_name.c_str());
//			message_route_table[pair.first].connect(pair.second);
//		}

//	} catch (pluginlib::PluginlibException &ex) {
//		ROS_ERROR_STREAM("Plugin [alias " << pl_name << "] load exception: " << ex.what());
//	}
//}

void MavRos::terminate_cb() {
    //ROS_ERROR("FCU connection closed, mavros will be terminated.");
    //ros::requestShutdown();
    mav_uas.stop();
}

void MavRos::startup_px4_usb_quirk(void) {
	/* sample code from QGC */
	const uint8_t init[] = {0x0d, 0x0d, 0x0d, 0};
	const uint8_t nsh[] = "sh /etc/init.d/rc.usb\n";

    std::cout << "Autostarting mavlink via USB on PX4" << std::endl;
	UAS_FCU(&mav_uas)->send_bytes(init, 3);
	UAS_FCU(&mav_uas)->send_bytes(nsh, sizeof(nsh) - 1);
	UAS_FCU(&mav_uas)->send_bytes(init, 4);	/* NOTE in original init[3] */
}

void MavRos::log_connect_change(bool connected) {
	/* note: sys_status plugin required */
	if (connected)
        ;//ROS_INFO("CON: Got HEARTBEAT, connected.");
	else
        ;//ROS_WARN("CON: Lost connection, HEARTBEAT timed out.");
}

