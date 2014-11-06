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

 File: libbarrett_ros.cpp
 Date: 15 October, 2014
 Author: Hariharasudan Malaichamee
 */

#include "ros/ros.h"
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <boost/thread.hpp>

#include <barrett_arm.h>
#include <barrett_hand.h>

const int ARM_RATE = 200;
using namespace barrett;
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
             systems::Wam<DOF>& wam) {
  ros::init(argc, argv, "libbarrett_ros");
  ros::NodeHandle nh;
  ros::Rate loop_rate(ARM_RATE);

  bool handIsFound = false;

  barm::BarrettArmInterface<DOF> barrett_arm(pm, wam, nh);
  bhand::BarrettHandInterface<DOF> barrett_hand(pm, wam, nh);

  barrett_arm.init();
  if (pm.foundHand()) {
    barrett_hand.init();
    handIsFound = true;
  }

  while (ros::ok()) {
    barrett_arm.armPublishInfo();
    if (handIsFound)
      barrett_hand.handPublishInfo();
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Create thread for the Barrett Arm
 /* boost::thread arm_thread(&barm::BarrettArmInterface<DOF>::start,
                           &barrett_arm);

  // Check if the hand is present and start a seperate thread for it
  if (pm.foundHand()) {
    boost::thread hand_thread(&bhand::BarrettHandInterface<DOF>::start,
                              &barrett_hand);
    hand_thread.join();
  }

  arm_thread.join();*/
  return 0;
}
