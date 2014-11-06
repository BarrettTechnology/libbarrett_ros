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

#ifndef INCLUDE_BARRETT_ARM_H_
#define INCLUDE_BARRETT_ARM_H_

#include "ros/ros.h"
#include <string>
#include <cstdlib>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>

#include <wam_msgs/EndpointState.h>
#include <wam_msgs/RTPosMode.h>
#include <wam_msgs/GravityComp.h>
#include <wam_msgs/JointMove.h>

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

using namespace barrett;

namespace barm {

// Subscribe topics
const std::string ARM_CONTROL_TOPIC = "arm/control";

// Publish topics
const std::string ARM_JS_TOPIC = "arm/joint_states";
const std::string ARM_ES_TOPIC = "arm/endpoint_state";

// Service topics
const std::string ARM_CONTROL_GRAV = "arm/gravity";
const std::string ARM_MOVE_HOME = "arm/move_home";
const std::string ARM_JOINT_MOVETO = "arm/joint_move_to";

const std::string jnt_names[] = { "j1", "j2", "j3", "j4", "j5", "j6", "j7" };

template<size_t DOF>
class BarrettArmInterface {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  // WAM information for retrieval
  jp_type jp_info, jp_cmd;
  jv_type jv_info;
  jt_type jt_info;

  cp_type cp_info;
  cv_type cv_info;
  pose_type pt_info;
  Eigen::Quaterniond qt_info;

  // WAM information for publishing
  sensor_msgs::JointState js_info;
  wam_msgs::EndpointState es_info;

  systems::Wam<DOF>* arm_wam;
  ProductManager* arm_pm;

  ros::NodeHandle* nh;
  ros::Subscriber arm_control_sub;
  ros::Publisher arm_js_pub, arm_es_pub;
  ros::ServiceServer arm_ctrl_gravity_srv, arm_mv_home_srv, arm_jnt_mv_to_srv;

  bool controlGravity(wam_msgs::GravityComp::Request &,
      wam_msgs::GravityComp::Response &);
  void armControlModes(const wam_msgs::RTPosMode&);
  bool armMoveHome(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool armJointMoveTo(wam_msgs::JointMove::Request &,
      wam_msgs::JointMove::Response &);

 public:
  BarrettArmInterface(ProductManager &pm, systems::Wam<DOF> &wam,
                      ros::NodeHandle &n)
  : arm_wam(&wam), arm_pm(&pm), nh(&n) {}
  void init();
  void armPublishInfo();
  ~BarrettArmInterface() {
    arm_wam->idle();
    arm_pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
  }
};

/*
 * Service call to turn on/off the gravity as requested
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::controlGravity(
    wam_msgs::GravityComp::Request &req, wam_msgs::GravityComp::Response &res) {

  arm_wam->gravityCompensate(req.gravity);
  ROS_INFO("Gravity Compensation Request: %s",
           (req.gravity) ? "true" : "false");
  return true;
}

template<size_t DOF>
void BarrettArmInterface<DOF>::armControlModes(const wam_msgs::RTPosMode& msg) {
  switch (msg.mode) {
    case wam_msgs::RTPosMode::JOINT_POSITION_CONTROL:
      ROS_INFO("Holding joint positions.");
      arm_wam->moveTo(arm_wam->getJointPositions());
      break;

    case wam_msgs::RTPosMode::CARTESIAN_POSITION_CONTROL:
      ROS_INFO("Holding tool position.");
      arm_wam->moveTo(arm_wam->getToolPosition());
      break;

    case wam_msgs::RTPosMode::CARTESIAN_ORIENTATION_CONTROL:
      ROS_INFO("Holding tool orientation.");
      arm_wam->moveTo(arm_wam->getToolOrientation());
      break;

    case wam_msgs::RTPosMode::CARTESIAN_POS_ORIENT_CONTROL:
      ROS_INFO("Holding both tool position and orientation.");
      arm_wam->moveTo(arm_wam->getToolPose());
      break;

    case wam_msgs::RTPosMode::GRAVITY_COMP:
      ROS_INFO("Gravity Compensation mode.");
      arm_wam->gravityCompensate();
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
void BarrettArmInterface<DOF>::armPublishInfo() {
  jp_info = arm_wam->getJointPositions();
  jv_info = arm_wam->getJointVelocities();
  jt_info = arm_wam->getJointTorques();

  cp_info = arm_wam->getToolPosition();
  cv_info = arm_wam->getToolVelocity();
  qt_info = arm_wam->getToolOrientation();

  js_info.position.resize(DOF);
  js_info.velocity.resize(DOF);
  js_info.effort.resize(DOF);

  es_info.positions.resize(3);
  es_info.velocities.resize(3);

  // Pack the Joint and Endpoint state messages with the updated values
  for (size_t i = 0; i < DOF; ++i) {
    js_info.position[i] = jp_info[i];
    js_info.velocity[i] = jv_info[i];
    js_info.effort[i] = jt_info[i];

    if (i < 3) {
      es_info.positions[i] = cp_info[i];
      es_info.velocities[i] = cv_info[i];
    }
  }
  es_info.orientation.x = qt_info.x();
  es_info.orientation.y = qt_info.y();
  es_info.orientation.z = qt_info.z();
  es_info.orientation.w = qt_info.w();

  arm_js_pub.publish(js_info);
  arm_es_pub.publish(es_info);
}

/*
 * Moves the arm to its home position
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armMoveHome(std_srvs::Empty::Request &req,
                                           std_srvs::Empty::Response &res) {
  ROS_INFO("Moving the arm to its home position.");
  arm_wam->moveHome();
  return true;
}

/*
 * Moves the arm joints to the commanded position
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armJointMoveTo(
    wam_msgs::JointMove::Request &req, wam_msgs::JointMove::Response &res) {

  if (req.joints.size() != DOF) {
    ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF",
             req.joints.size(), DOF);
    return false;
  }

  ROS_INFO("Moving the arm joints to Commanded Joint Position");
  for (size_t i = 0; i < DOF; i++)
    jp_cmd[i] = req.joints[i];
  arm_wam->moveTo(jp_cmd, false);
  return true;
}
/*
 * Initialize the subscribers and publishers. Initialize the messages with their default values
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::init() {
  arm_control_sub = nh->subscribe(ARM_CONTROL_TOPIC, 1,
                                  &BarrettArmInterface<DOF>::armControlModes,
                                  this);

  // Joint Publishers
  arm_js_pub = nh->advertise<sensor_msgs::JointState>(ARM_JS_TOPIC, 1);

  // Cartesian Publishers
  arm_es_pub = nh->advertise<wam_msgs::EndpointState>(ARM_ES_TOPIC, 1);

  // Advertise the following services
  arm_ctrl_gravity_srv = nh->advertiseService(
      ARM_CONTROL_GRAV, &BarrettArmInterface<DOF>::controlGravity, this);
  arm_mv_home_srv = nh->advertiseService(ARM_MOVE_HOME,
                                         &BarrettArmInterface<DOF>::armMoveHome,
                                         this);
  arm_jnt_mv_to_srv = nh->advertiseService(
      ARM_JOINT_MOVETO, &BarrettArmInterface<DOF>::armJointMoveTo, this);

  // Initialize the Joint state publisher message with its default values
  js_info.name.resize(DOF);
  for (size_t i = 0; i < DOF; ++i) {
    js_info.name[i] = jnt_names[i];
  }
  // Turn on the gravity compensation by default
  arm_wam->gravityCompensate();
}
}  // namespace
#endif  // INCLUDE_BARRETT_ARM_H_
