/*
 * barrett_arm.h
 *
 *  Created on: Oct 16, 2014
 *      Author: robot
 */

#ifndef BARRETT_ARM_H_
#define BARRETT_ARM_H_

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <wam_msgs/EndpointState.h>

#include <wam_msgs/RTPosMode.h>

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>

using namespace barrett;
const std::string BARRETT_ARM_CONTROL_TOPIC = "barrett/arm/control";
const std::string BARRETT_ARM_JS_TOPIC = "barrett/arm/joint_state";
const std::string BARRETT_ARM_ES_TOPIC = "barrett/arm/endpoint_state";

const std::string jnt_names[] = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

template<size_t DOF>
class barrett_arm_interface{
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
	ros::Subscriber barrett_arm_control_sub;
	ros::Publisher barrett_arm_js_pub, barrett_arm_es_pub;

	void barrett_arm_control_modes(const wam_msgs::RTPosMode& msg);
	void publish_barrett_arm_info();

public:
	barrett_arm_interface(ProductManager &pm, systems::Wam<DOF> &wam): arm_wam(&wam), arm_pm(&pm){};
	void start();

};

template<size_t DOF>
void barrett_arm_interface<DOF>::barrett_arm_control_modes(const wam_msgs::RTPosMode& msg){
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
void barrett_arm_interface<DOF>::publish_barrett_arm_info(){

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

	barrett_arm_js_pub.publish(js);
	barrett_arm_es_pub.publish(es);
}

/*
 * Initialize the subscribers and publishers. Initialize the messages with their default values
 */
template<size_t DOF>
void barrett_arm_interface<DOF>::start(){

	barrett_arm_control_sub = nh.subscribe(BARRETT_ARM_CONTROL_TOPIC, 1, &barrett_arm_interface<DOF>::barrett_arm_control_modes, this);

	//Joint Publishers
	barrett_arm_js_pub = nh.advertise<sensor_msgs::JointState>(BARRETT_ARM_JS_TOPIC, 1);

	//Cartesian Publishers
	barrett_arm_es_pub = nh.advertise<wam_msgs::EndpointState>(BARRETT_ARM_ES_TOPIC, 1);

	//Initialize the Joint state publisher message with its default values
	for (size_t i=0; i<DOF; ++i){
		js.name[i] = jnt_names[i];
	}

	//Set the loop rate at 200 Hz
	ros::Rate loop_rate(200);

	//Turn on the gravity compensation by default
	arm_wam->gravityCompensate();

	while(ros::ok){
		publish_barrett_arm_info();
		ros::spinOnce();
		loop_rate.sleep();
	}

	//Idle the Wam
	arm_wam->idle();
	arm_pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
}

#endif /* BARRETT_ARM_H_ */
