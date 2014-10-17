/*
 * libbarrett_ros.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: Hariharasudan Malaichamee
 */
#include "ros/ros.h"
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <boost/thread.hpp>

#include "../include/barrett_arm.h"

using namespace barrett;
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	ros::init(argc, argv, "libbarrett_interface");

	barrett_arm_interface<DOF> barrett_arm( pm, wam);

	//Create thread for the individual products
	boost::thread arm_thread(&barrett_arm_interface<DOF>::start, &barrett_arm);

	arm_thread.join();
	return 0;
}
