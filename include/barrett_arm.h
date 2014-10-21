/*
 Copyright 2012 Barrett Technology <support@barrett.com>

 This file is part of libbarrett_ros.

 This version of libbarrett_ros is free software: you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version.

 This version of libbarrett_ros is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this version of libbarrett_ros.  If not, see
 <http://www.gnu.org/licenses/>.

 Barrett Technology holds all copyrights on libbarrett_ros. As the sole
 copyright holder, Barrett reserves the right to release future versions
 of libbarrett_ros under a different license.

 File: barrett_arm.h
 Date: 15 October, 2014
 Author: Hariharasudan Malaichamee
 */

#ifndef BARRETT_ARM_H_
#define BARRETT_ARM_H_

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cstdlib>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <wam_msgs/EndpointState.h>

#include <wam_msgs/RTPosMode.h>

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

using namespace barrett;

namespace barm{

const int ARM_RATE = 200; //Execution rate in Hz

const std::string ARM_CONTROL_TOPIC = "arm/control";
const std::string ARM_JS_TOPIC = "arm/joint_states";
const std::string ARM_ES_TOPIC = "arm/endpoint_state";

const std::string jnt_names[] = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

template<size_t DOF>
class BarrettArmInterface{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	//WAM information for retrieval
	jp_type jp;
	jv_type jv;
	jt_type jt;

	cp_type cp;
	cv_type cv;
	pose_type pt;
	Eigen::Quaterniond qt;

	//WAM information for publishing
	sensor_msgs::JointState js;
	wam_msgs::EndpointState es;

	systems::Wam<DOF>* arm_wam;
	ProductManager* arm_pm;
	ros::NodeHandle nh;
	ros::Subscriber arm_control_sub;
	ros::Publisher arm_js_pub, arm_es_pub;

	void armControlModes(const wam_msgs::RTPosMode&);
	void armPublishInfo();

public:
	BarrettArmInterface(ProductManager &pm, systems::Wam<DOF> &wam): arm_wam(&wam), arm_pm(&pm){};
	void start();

};

template<size_t DOF>
void BarrettArmInterface<DOF>::armControlModes(const wam_msgs::RTPosMode& msg){
		switch (msg.mode) {
		case wam_msgs::RTPosMode::JOINT_POSITION_CONTROL:
			printf("Holding joint positions.\n");
			arm_wam->moveTo(arm_wam->getJointPositions());
			break;

		case wam_msgs::RTPosMode::CARTESIAN_POSITION_CONTROL:
			printf("Holding tool position.\n");
			arm_wam->moveTo(arm_wam->getToolPosition());
			break;

		case wam_msgs::RTPosMode::CARTESIAN_ORIENTATION_CONTROL:
			printf("Holding tool orientation.\n");
			arm_wam->moveTo(arm_wam->getToolOrientation());
			break;

		case wam_msgs::RTPosMode::CARTESIAN_POS_ORIENT_CONTROL:
			printf("Holding both tool position and orientation.\n");
			arm_wam->moveTo(arm_wam->getToolOrientation());
			break;

		case wam_msgs::RTPosMode::GRAVITY_COMP:
			std::cout << "Moving to home position: "<< std::endl;
			arm_wam->moveHome();
			break;

		default:
			printf("Unrecognized option.\n");
			break;
		}
	}

/*
 * Publish the Joint and the endpoint states of the wam
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armPublishInfo(){

	jp = arm_wam->getJointPositions();
	jv = arm_wam->getJointVelocities();
	jt = arm_wam->getJointTorques();

	cp = arm_wam->getToolPosition();
	cv = arm_wam->getToolVelocity();
	qt = arm_wam->getToolOrientation();

	//Pack the Joint and Endpoint state messages with the updated values
	for(size_t i = 0; i < DOF; ++i){
		js.position[i] = jp[i];
		js.velocity[i] = jv[i];
		js.effort[i] = jt[i];

		if(i<3){
		es.positions[i] = cp[i];
		es.velocities[i] = cv[i];
		}
	}
	es.orientation.x = qt.x();
	es.orientation.y = qt.y();
	es.orientation.z = qt.z();
	es.orientation.w = qt.w();

	arm_js_pub.publish(js);
	arm_es_pub.publish(es);
}

/*
 * Initialize the subscribers and publishers. Initialize the messages with their default values
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::start(){

	arm_control_sub = nh.subscribe(ARM_CONTROL_TOPIC, 1, &BarrettArmInterface<DOF>::armControlModes, this);

	//Joint Publishers
	arm_js_pub = nh.advertise<sensor_msgs::JointState>(ARM_JS_TOPIC, 1);

	//Cartesian Publishers
	arm_es_pub = nh.advertise<wam_msgs::EndpointState>(ARM_ES_TOPIC, 1);

	//Initialize the Joint state publisher message with its default values
	for (size_t i=0; i<DOF; ++i){
		js.name[i] = jnt_names[i];
	}

	//Set the loop rate at 200 Hz
	ros::Rate loop_rate(ARM_RATE);

	//Turn on the gravity compensation by default
	arm_wam->gravityCompensate();

	while(ros::ok){
		armPublishInfo();
		ros::spinOnce();
		loop_rate.sleep();
	}

	//Idle the Wam
	arm_wam->idle();
	arm_pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
}
} //namespace
#endif /* BARRETT_ARM_H_ */
