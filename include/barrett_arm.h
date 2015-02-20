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

#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>

#include <wam_msgs/EndpointState.h>
#include <wam_msgs/TurnOn.h>
#include <wam_msgs/Update.h>
#include <wam_msgs/GravityComp.h>
#include <wam_msgs/JointMove.h>
#include <wam_msgs/PID.h>
#include <wam_msgs/RTCartVel.h>
#include <wam_msgs/RTOrtnVel.h>
#include <wam_msgs/RTJointVel.h>
#include <wam_msgs/RTJointPos.h>
#include <wam_msgs/RTJointTq.h>
#include <wam_msgs/RTCartPos.h>
#include <wam_msgs/RTSelect.h>
#include <wam_msgs/SafetyState.h>
#include <wam_msgs/DOF.h>
#include <wam_msgs/SafetyLimits.h>

#include <wam_control.h>

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

using namespace barrett;

namespace barm {

// Subscribe topics
const std::string ARM_UPDATE_JNTPOSGAINS_TOPIC = "arm/update_joint_pos_gains";
const std::string ARM_UPDATE_JNTVELGAINS_TOPIC = "arm/update_joint_vel_gains";
const std::string ARM_UPDATE_CARTPOSGAINS_TOPIC = "arm/update_cart_pos_gains";
const std::string ARM_UPDATE_CARTPOSEGAINS_TOPIC = "arm/update_cart_pose_gains";
const std::string ARM_UPDATE_CARTVEL_TOPIC = "arm/cart_vel_command";
const std::string ARM_UPDATE_JNTVEL_TOPIC = "arm/joint_vel_command";
const std::string ARM_UPDATE_ORTNVEL_TOPIC = "arm/ortn_vel_command";
const std::string ARM_UPDATE_CARTPOS_TOPIC = "arm/cart_pos_command";
const std::string ARM_UPDATE_JNTPOS_TOPIC = "arm/joint_pos_command";
const std::string ARM_UPDATE_JNTTRQ_TOPIC = "arm/joint_trq_command";

// Publish topics
const std::string ARM_JS_TOPIC = "arm/joint_states";
const std::string ARM_ES_TOPIC = "arm/endpoint_state";

// Service topics
const std::string ARM_TOGGLE_GRAV = "arm/toggle_gravity";
const std::string ARM_UPDATE_GRAV = "arm/update_gravity";
const std::string ARM_HOLD_JNTPOS_TOPIC = "arm/hold_joint_pos";
const std::string ARM_HOLD_CARTPOS_TOPIC = "arm/hold_cart_pos";
const std::string ARM_HOLD_CARTPOSE_TOPIC = "arm/hold_cart_pose";
const std::string ARM_MOVE_HOME_TOPIC = "arm/move_home";
const std::string ARM_JOINT_MOVETO_TOPIC = "arm/joint_move_to";
const std::string ARM_IDLE_TOPIC = "arm/idle";
const std::string ARM_SELECT_RT_MODE_TOPIC = "arm/select_rt_control_mode";
const std::string ARM_GET_DOF_TOPIC = "arm/getDOF";
const std::string ARM_UPDATE_SAFETY_LIMITS_TOPIC = "arm/update_safety_limits";
const std::string ARM_SAFETY_STATE_TOPIC = "arm/safety_state";
const std::string ARM_SET_MOVE_VEL_TOPIC = "arm/set_move_velocity";
const std::string ARM_SET_MOVE_ACCEL_TOPIC = "arm/set_move_acceleration";
const std::string ARM_APPLY_ACCEL_VEL_TOPIC =
    "arm/apply_move_acceleration_velocity";

const std::string jnt_names[] = { "j1", "j2", "j3", "j4", "j5", "j6", "j7" };

const double SPEED = 0.03;  // Default Cartesian Velocity

enum RT_mode_t {
  CARTESIAN_VELOCITY = 1,
  JOINT_VELOCITY = 2,
  ORIENTATION_VELOCITY = 3,
  CARTESIAN_POSITION = 4,
  JOINT_POSITION = 5,
  JOINT_TORQUE = 6
};

// Creating a templated multiplier for our real-time computation
template<typename T1, typename T2, typename OutputType>
class Multiplier : public systems::System, public systems::SingleOutput<
    OutputType> {
 public:
  Input<T1> input1;
 public:
  Input<T2> input2;

 public:
  explicit Multiplier(std::string sysName = "Multiplier")
      : systems::System(sysName),
        systems::SingleOutput<OutputType>(this),
        input1(this),
        input2(this) {
  }
  virtual ~Multiplier() {
    mandatoryCleanUp();
  }

 protected:
  OutputType data;
  virtual void operate() {
    data = input1.getValue() * input2.getValue();
    this->outputValue->setData(&data);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(Multiplier);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Creating a templated converter from Roll, Pitch, Yaw to Quaternion for real-time computation
class ToQuaternion : public systems::SingleIO<math::Vector<3>::type,
    Eigen::Quaterniond> {
 public:
  Eigen::Quaterniond outputQuat;

 public:
  explicit ToQuaternion(std::string sysName = "ToQuaternion")
      : systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>(sysName) {
  }
  virtual ~ToQuaternion() {
    mandatoryCleanUp();
  }

 protected:
  tf::Quaternion q;
  virtual void operate() {
    const math::Vector<3>::type &inputRPY = input.getValue();
    q.setEulerZYX(inputRPY[2], inputRPY[1], inputRPY[0]);
    outputQuat.x() = q.getX();
    outputQuat.y() = q.getY();
    outputQuat.z() = q.getZ();
    outputQuat.w() = q.getW();
    this->outputValue->setData(&outputQuat);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Simple Function for converting Quaternion to RPY
math::Vector<3>::type toRPY(Eigen::Quaterniond inquat) {
  math::Vector<3>::type newRPY;
  tf::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
  tf::Matrix3x3(q).getEulerZYX(newRPY[2], newRPY[1], newRPY[0]);
  return newRPY;
}

template<size_t DOF>
class BarrettArmInterface {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  bool cart_vel_status, ortn_vel_status, jnt_vel_status, jnt_trq_status,
  jnt_pos_status, cart_pos_status, ortn_pos_status, new_rt_cart_vel_cmd,
  new_rt_ortn_vel_cmd, new_rt_jnt_vel_cmd, new_rt_jnt_trq_cmd,
  new_rt_jnt_pos_cmd, new_rt_cart_pos_cmd;
  double cart_vel_mag, ortn_vel_mag, move_vel, move_accel;

  math::Vector<3>::type rt_ortn_cmd;

  // WAM information for retrieval
  jp_type jp_info, jp_cmd, jp_kp_gains, jp_ki_gains, jp_kd_gains,
  rt_jp_cmd, rt_jp_rl;
  jv_type jv_info, jv_kp_gains, jv_ki_gains, jv_kd_gains, rt_jv_cmd;
  jt_type jt_info, rt_jt_cmd;

  cp_type cp_info, cp_kp_gains, cp_ki_gains, cp_kd_gains, rt_cv_cmd,
  rt_cp_rl, rt_cp_cmd;
  cv_type cv_info;
  pose_type pt_info;
  Eigen::Quaterniond qt_info;

  systems::ExposedOutput<cp_type> cart_dir, current_cart_pos, cp_track;
  systems::ExposedOutput<Eigen::Quaterniond> current_ortn;
  systems::Ramp ramp;
  systems::Summer<cp_type> cart_pos_sum;
  systems::TupleGrouper<cp_type, Eigen::Quaterniond> rt_pose_cmd;
  systems::ExposedOutput<math::Vector<3>::type> rpy_cmd, current_rpy_ortn;
  systems::Summer<math::Vector<3>::type> ortn_cmd_sum;
  systems::ExposedOutput<jv_type> jv_track;
  systems::ExposedOutput<jp_type> jp_track;
  systems::ExposedOutput<jt_type> jt_track;
  systems::RateLimiter<jp_type> jp_rl;
  systems::RateLimiter<cp_type> cp_rl;

  Multiplier<double, cp_type, cp_type> mult_linear;
  Multiplier<double, math::Vector<3>::type, math::Vector<3>::type> mult_angular;

  ToQuaternion to_quat, to_quat_print;

  // WAM information for publishing
  sensor_msgs::JointState js_info;
  wam_msgs::EndpointState es_info;
  //wam_msgs::SafetyState safety_st;

  wam_msgs::RTSelect::Request active_RT_mode;

  systems::Wam<DOF>* arm_wam;
  ProductManager* arm_pm;
  SafetyModule& sm;
  SafetyModule::PendantState ps;
  BarrettBot<DOF>* bbot;

  ros::NodeHandle* nh;
  ros::Subscriber arm_jnt_pos_gns_sub, arm_jnt_vel_gns_sub,
  arm_cart_pos_gns_sub, arm_cart_pose_gns_sub, arm_cart_vel_cmd_sub,
  arm_jnt_vel_cmd_sub, arm_ortn_vel_cmd_sub, arm_cart_pos_cmd_sub,
  arm_jnt_pos_cmd_sub, arm_jnt_trq_cmd_sub;

  ros::Publisher arm_js_pub, arm_es_pub;
  ros::ServiceServer arm_toggle_gravity_srv, arm_update_gravity_srv,
  arm_hold_jntpos_srv, arm_hold_cartpos_srv, arm_hold_cartpose_srv,
  arm_mv_home_srv, arm_jnt_mv_to_srv, arm_idle_srv, arm_select_rt_mode_srv,
  arm_get_DOF_srv, arm_update_safety_lts_srv, arm_get_safety_srv,
  arm_set_move_vel_srv, arm_set_move_accel_srv, arm_apply_vel_accel_srv;

  ros::Time last_cart_vel_msg_time, last_ortn_vel_msg_time,
  last_jnt_vel_msg_time, last_jnt_pos_msg_time, last_cart_pos_msg_time,
  last_ortn_pos_msg_time, last_jnt_trq_msg_time;

  ros::Duration rt_msg_timeout;

  // Subscriber Callback functions
  void armUpdateJntPosGains(const wam_msgs::PID &);
  void armUpdateJntVelGains(const wam_msgs::PID &);
  void armUpdateCartPosGains(const wam_msgs::PID &);
  void armUpdateCartPoseGains(const wam_msgs::PID &);
  void armUpdateCartVel(const wam_msgs::RTCartVel::ConstPtr &);
  void armUpdateJntVel(const wam_msgs::RTJointVel::ConstPtr &);
  void armUpdateOrtnVel(const wam_msgs::RTOrtnVel::ConstPtr &);
  void armUpdateCartPos(const wam_msgs::RTCartPos::ConstPtr &);
  //void armUpdateJntPos(const wam_msgs::RTJointPos::ConstPtr &);
  void armUpdateJntTrq(const wam_msgs::RTJointTq::ConstPtr &);

  // Services Callback functions
  bool armToggleGravity(wam_msgs::TurnOn::Request &,
      wam_msgs::TurnOn::Response &);
  bool armUpdateGravity(wam_msgs::Update::Request &,
      wam_msgs::Update::Response &);
  bool armHoldJointPos(wam_msgs::TurnOn::Request &,
      wam_msgs::TurnOn::Response &);
  bool armHoldCartPos(wam_msgs::TurnOn::Request &,
      wam_msgs::TurnOn::Response &);
  bool armHoldCartPose(wam_msgs::TurnOn::Request &,
      wam_msgs::TurnOn::Response &);
  bool armMoveHome(std_srvs::Empty::Request &,
      std_srvs::Empty::Response &);
  bool armJointMoveTo(wam_msgs::JointMove::Request &,
      wam_msgs::JointMove::Response &);
  bool armIdle(std_srvs::Empty::Request &,
      std_srvs::Empty::Response &);
  bool armGetDOF(wam_msgs::DOF::Request &, wam_msgs::DOF::Response &);
  bool armSelectRTMode(wam_msgs::RTSelect::Request &,
      wam_msgs::RTSelect::Response &);
  bool armUpdateSafetyLts(wam_msgs::SafetyLimits::Request &,
      wam_msgs::SafetyLimits::Response &);
  bool armGetSafetyState(wam_msgs::SafetyState::Request &,
      wam_msgs::SafetyState::Response &);
  bool armSetMoveVel(wam_msgs::Update::Request &,
      wam_msgs::Update::Response &);
  bool armSetMoveAccel(wam_msgs::Update::Request &,
      wam_msgs::Update::Response &);
  bool armApplyVelAccel(std_srvs::Empty::Request &,
      std_srvs::Empty::Response &);

  void armHoldCurState();
  void armUpdateRT();

public:
  BarrettArmInterface(ProductManager &pm, systems::Wam<DOF> &wam,
      ros::NodeHandle &n):jnt_pos_status(false), move_vel(-1234), move_accel(-1234),
  ramp(NULL, SPEED), arm_wam(&wam), arm_pm(&pm),
  sm(*arm_pm->getSafetyModule()), nh(&n) {}
  void init();
  void armPublishInfo();
  void armUpdateCmds();
  ~BarrettArmInterface() {
    arm_wam->idle();
    arm_pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
  }
};

template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateJntPosGains(const wam_msgs::PID &msg) {
  if (msg.kp.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      jp_kp_gains[i] = msg.kp[i];
    arm_wam->jpController.setKp(jp_kp_gains);
  } else {
    ROS_ERROR("Unable to update Joint Positions' Kp:"
             " The gains sent must be >= to the DOF");
  }
  if (msg.ki.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      jp_ki_gains[i] = msg.ki[i];
    arm_wam->jpController.setKi(jp_ki_gains);
  } else {
    ROS_ERROR("Unable to update Joint Positions' Ki:"
             " The gains sent must be >= to the DOF");
  }
  if (msg.kd.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      jp_kd_gains[i] = msg.kd[i];
    arm_wam->jpController.setKd(jp_kd_gains);
  } else {
    ROS_ERROR("Unable to update Joint Positions' Kd:"
             " The gains sent must be >= to the DOF");
  }
}

template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateJntVelGains(const wam_msgs::PID &msg) {
  if (msg.kp.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      jv_kp_gains[i] = msg.kp[i];
    arm_wam->jvController1.setKp(jv_kp_gains);
  } else {
    ROS_INFO("Unable to update Joint Velocities' Kp:"
             " The gains sent must be >= to the DOF");
  }
  if (msg.ki.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      jv_ki_gains[i] = msg.ki[i];
    arm_wam->jvController1.setKi(jv_ki_gains);
  } else {
    ROS_INFO("Unable to update Joint Velocities' Ki:"
             " The gains sent must be >= to the DOF");
  }
  if (msg.kd.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      jv_kd_gains[i] = msg.kd[i];
    arm_wam->jvController1.setKd(jv_kd_gains);
  } else {
    ROS_INFO("Unable to update Joint Velocities' Kd:"
             " The gains sent must be >= to the DOF");
  }
}

template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateCartPosGains(const wam_msgs::PID &msg) {
  if (msg.kp.size() >= 3) {
    for (size_t i = 0; i < 3; ++i)
      cp_kp_gains[i] = msg.kp[i];
    arm_wam->tpController.setKp(cp_kp_gains);
  } else {
    ROS_INFO("Unable to update Cartesian Positions' Kp:"
             " The no of gains sent must be = 3");
  }
  if (msg.ki.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      cp_ki_gains[i] = msg.ki[i];
    arm_wam->tpController.setKi(cp_ki_gains);
  } else {
    ROS_INFO("Unable to update Cartesian Positions' Ki:"
             " The no of gains sent must be = 3");
  }
  if (msg.kd.size() >= DOF) {
    for (size_t i = 0; i < DOF; ++i)
      cp_kd_gains[i] = msg.kd[i];
    arm_wam->tpController.setKd(cp_kd_gains);
  } else {
    ROS_INFO("Unable to update Cartesian Positions' Kd:"
             " The no of gains sent must be = 3");
  }
}

template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateCartPoseGains(
    const wam_msgs::PID &msg) {
  if (msg.kp.size() >= 1) {
    arm_wam->toController.setKp(msg.kp[0]);
  } else {
    ROS_WARN("Unable to update Pose Kp: The no of gains sent must be = 1");
  }
  if (msg.kd.size() >= 1) {
    arm_wam->toController.setKd(msg.kd[0]);
  } else {
    ROS_INFO("Unable to update Pose Kd: The no of gains sent must be = 1");
  }
}

/*
 * Callback function for RT Cartesian Velocity messages
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateCartVel(
    const wam_msgs::RTCartVel::ConstPtr& msg) {
  if (cart_vel_status) {
    for (size_t i = 0; i < 3; i++)
      rt_cv_cmd[i] = msg->direction[i];
    new_rt_cart_vel_cmd = true;
    if (msg->magnitude != 0)
      cart_vel_mag = msg->magnitude;
  }
  last_cart_vel_msg_time = ros::Time::now();

}

/*
 * Callback function for RT Joint Velocity Messages
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateJntVel(
    const wam_msgs::RTJointVel::ConstPtr& msg) {
  if (msg->velocities.size() != DOF) {
    ROS_INFO("Commanded Joint Velocities != DOF of WAM");
    return;
  }
  if (jnt_vel_status) {
    for (size_t i = 0; i < DOF; i++)
      rt_jv_cmd[i] = msg->velocities[i];
    new_rt_jnt_vel_cmd = true;
  }
  last_jnt_vel_msg_time = ros::Time::now();
}

/*
 * Callback function for RT Orientation Velocity Messages
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateOrtnVel(
    const wam_msgs::RTOrtnVel::ConstPtr& msg) {
  if (ortn_vel_status) {
    for (size_t i = 0; i < 3; i++)
      rt_ortn_cmd[i] = msg->angular[i];
    new_rt_ortn_vel_cmd = true;
    if (msg->magnitude != 0)
      ortn_vel_mag = msg->magnitude;
  }
  last_ortn_vel_msg_time = ros::Time::now();
}

/*
 * Callback function for RT Cartesian Position Messages
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateCartPos(
    const wam_msgs::RTCartPos::ConstPtr& msg) {
  if (cart_pos_status) {
    for (size_t i = 0; i < 3; i++) {
      rt_cp_cmd[i] = msg->position[i];
      rt_cp_rl[i] = msg->rate_limits[i];
    }
    new_rt_cart_pos_cmd = true;
  }
  last_cart_pos_msg_time = ros::Time::now();
}

/*
 * Callback function for RT Joint Position Messages
 */
/*template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateJntPos(
    const wam_msgs::RTJointPos::ConstPtr& msg) {
  if (msg->joints.size() != DOF) {
    ROS_INFO("Commanded Joint Positions != DOF of WAM");
    return;
  }
  if (jnt_pos_status) {
    for (size_t i = 0; i < DOF; i++) {
      rt_jp_cmd[i] = msg->joints[i];
      rt_jp_rl[i] = msg->rate_limits[i];
    }
    new_rt_jnt_pos_cmd = true;
  }
  last_jnt_pos_msg_time = ros::Time::now();
}*/

/*
 *  Callback function for RT Cartesian Position Messages
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateJntTrq(
    const wam_msgs::RTJointTq::ConstPtr& msg) {
  if (jnt_trq_status) {
    for (size_t i = 0; i < 3; i++) {
      rt_jt_cmd[i] = msg->torques[i];
    }
    new_rt_jnt_trq_cmd = true;
  }
  last_jnt_trq_msg_time = ros::Time::now();
}

/*
 * Service call to turn on/off the gravity as requested
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armToggleGravity(
    wam_msgs::TurnOn::Request &req, wam_msgs::TurnOn::Response &res) {

  arm_wam->gravityCompensate(req.turnOn);
  ROS_INFO("Gravity Compensation Request: %s", (req.turnOn) ? "true" : "false");
  return true;
}

/*
 * Service call to update the gravity as requested
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armUpdateGravity(
    wam_msgs::Update::Request &req, wam_msgs::Update::Response &res) {

  if (arm_wam->updateGravity(req.val))
    ROS_INFO("Gravity was updated with the value: %f", req.val);
  return true;
}

/*
 * Service call to turn on/off the Hold Joint Position demo
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armHoldJointPos(
    wam_msgs::TurnOn::Request &req, wam_msgs::TurnOn::Response &res) {
  if (req.turnOn)
    arm_wam->moveTo(arm_wam->getJointPositions());
  else
    arm_wam->idle();
  return true;
}

/*
 * Service call to turn on/off the Hold Cartesian Position demo
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armHoldCartPos(wam_msgs::TurnOn::Request &req,
                                              wam_msgs::TurnOn::Response &res) {
  if (req.turnOn)
    arm_wam->moveTo(arm_wam->getToolPosition());
  else
    arm_wam->idle();
  return true;
}

/*
 * Service call to turn on/off the Hold Cartesian Pose demo
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armHoldCartPose(
    wam_msgs::TurnOn::Request &req, wam_msgs::TurnOn::Response &res) {
  if (req.turnOn)
    arm_wam->moveTo(arm_wam->getToolOrientation());
  else
    arm_wam->idle();
  return true;
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

    //Populate the bbot class variables with the state information to be published via ros interface
    bbot->pos[i] = jp_info[i];
    bbot->vel[i] = jv_info[i];
    bbot->eff[i] = jt_info[i];

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
 * Update the Joint command variables
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateCmds() {
for(size_t i = 0; i < DOF ; ++i)
  rt_jp_cmd[i] = bbot->pos_cmd[i];
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
 * Moves the arm joints to the commanded position
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armIdle(std_srvs::Empty::Request &req,
                                       std_srvs::Empty::Response &res) {
  arm_wam->idle();
  ROS_INFO("The Arm is idled");
  return true;
}

/*
 * Returns the DOF of the robot
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armGetDOF(wam_msgs::DOF::Request &req,
                                         wam_msgs::DOF::Response &res) {
  res.DOF = DOF;
  return true;
}

/*
 * Switches the RT mode to the requested mode
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armSelectRTMode(
    wam_msgs::RTSelect::Request &req, wam_msgs::RTSelect::Response &res) {
  active_RT_mode = req;
  ROS_INFO("The RT mode is switched to %d", req.mode);
  return true;
}

/*
 * Applies the safety velocity and Torque Limits of the arm as requested
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armUpdateSafetyLts(
    wam_msgs::SafetyLimits::Request &req,
    wam_msgs::SafetyLimits::Response &res) {
  if (req.VL2 < 0)
    ROS_ERROR("The Velocity fault must be greater than or equal to 0 m/s");
  else {
    sm.setVelocityLimit(req.VL2, req.VL1);
    ROS_INFO(
        "The Updated Velocity warning is %f m/s and the Velocity fault is %f m/s",
        req.VL1, req.VL2);
  }
  if (req.TL1 <= 0)
    ROS_ERROR("The Torque fault must be greater than 0 N/m");
  else {
    sm.setTorqueLimit(req.TL2, req.TL1);
    ROS_INFO(
        "The Updated Torque warning is %f N/m and the Torque fault is %f N/m",
        req.TL1, req.TL2);
  }
}

/*
 * Retrieves the Safety Information about the robot
 */
template<size_t DOF>
bool BarrettArmInterface<DOF>::armGetSafetyState(
    wam_msgs::SafetyState::Request &req, wam_msgs::SafetyState::Response &res) {
  // Retrieve the Safety State of the Robot
  res.robot_state = arm_pm->getSafetyModule()->getMode();
  sm.getPendantState(&ps);
  res.Button_Pressed = ps.pressedButton;
  res.Velocity = ps.safetyParameters[0];
  res.Torque = ps.safetyParameters[1];
  res.Voltage = ps.safetyParameters[2];
  res.Heartbeat = ps.safetyParameters[3];
  res.Other = ps.safetyParameters[4];

  ROS_INFO("The current state of the Safety Module is updated in the response");

  return true;
}

template<size_t DOF>
bool BarrettArmInterface<DOF>::armSetMoveVel(wam_msgs::Update::Request &req,
                                             wam_msgs::Update::Response &res) {
  //Update the local variable with the requested move velocity
  move_vel = req.val;
  ROS_INFO(
      "Move acceleration is set to %f .Invoke the service %s to apply the requested changes",
      move_vel, ARM_SET_MOVE_VEL_TOPIC.c_str());

  return true;
}

template<size_t DOF>
bool BarrettArmInterface<DOF>::armSetMoveAccel(
    wam_msgs::Update::Request &req, wam_msgs::Update::Response &res) {
  //Update the local variable with the requested move velocity
  move_accel = req.val;
  ROS_INFO(
      "Move acceleration is set to %f .Invoke the service %s to apply the requested changes",
      move_accel, ARM_SET_MOVE_ACCEL_TOPIC.c_str());

  return true;
}

template<size_t DOF>
bool BarrettArmInterface<DOF>::armApplyVelAccel(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  // Apply the requested velocity and Acceleration
  if (move_accel != -1234 && move_vel != -1234) {
    arm_wam->moveTo(arm_wam->getJointPositions(), true, move_vel, move_accel);
    ROS_INFO("The velocity is updated as %f and the acceleration as %f",
             move_vel, move_accel);
    move_vel = move_accel = -1234;
  } else if (move_vel != -1234 && move_accel == -1234) {
    arm_wam->moveTo(arm_wam->getJointPositions(), true, move_vel);
    ROS_INFO("The velocity is updated as %f", move_vel);
    move_vel = -1234;
  } else {
    ROS_ERROR(
        "Set either the velocity or both, velocity and acceleration using %s and %s before invoking this service",
        ARM_SET_MOVE_VEL_TOPIC.c_str(), ARM_SET_MOVE_ACCEL_TOPIC.c_str());
    return false;
  }
  return true;
}

template<size_t DOF>
void BarrettArmInterface<DOF>::armHoldCurState() {
  if (cart_vel_status | ortn_vel_status | jnt_vel_status | jnt_pos_status
      | cart_pos_status) {
    arm_wam->moveTo(arm_wam->getJointPositions());  // Holds current joint positions upon a RT message timeout
    cart_vel_status = ortn_vel_status = jnt_vel_status = jnt_pos_status =
        cart_pos_status = ortn_pos_status = false;
  }
}
/*
 * Function to update the real-time control loops
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::armUpdateRT()  // systems::PeriodicDataLogger<debug_tuple>& logger
{
  switch (active_RT_mode.mode) {

    case CARTESIAN_VELOCITY:

      // Real-Time Cartesian Velocity Control Portion
      if (last_cart_vel_msg_time + rt_msg_timeout > ros::Time::now())  // checking if a cartesian velocity message has been published and if it is within timeout
          {
        if (!cart_vel_status) {
          cart_dir.setValue(cp_type(0.0, 0.0, 0.0));  // zeroing the cartesian direction
          current_cart_pos.setValue(arm_wam->getToolPosition());  // Initializing the cartesian position
          current_ortn.setValue(arm_wam->getToolOrientation());  // Initializing the orientation
          systems::forceConnect(ramp.output, mult_linear.input1);  // connecting the ramp to multiplier
          systems::forceConnect(cart_dir.output, mult_linear.input2);  // connecting the direction to the multiplier
          systems::forceConnect(mult_linear.output, cart_pos_sum.getInput(0));  // adding the output of the multiplier
          systems::forceConnect(current_cart_pos.output,
                                cart_pos_sum.getInput(1));  // with the starting cartesian position offset
          systems::forceConnect(cart_pos_sum.output, rt_pose_cmd.getInput<0>());  // saving summed position as new commanded pose.position
          systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>());  // saving the original orientation to the pose.orientation
          ramp.setSlope(cart_vel_mag);  // setting the slope to the commanded magnitude
          ramp.stop();  // ramp is stopped on startup
          ramp.setOutput(0.0);  // ramp is re-zeroed on startup
          ramp.start();  // start the ramp
          arm_wam->trackReferenceSignal(rt_pose_cmd.output);  // command WAM to track the RT commanded (500 Hz) updated pose
        } else if (new_rt_cart_vel_cmd) {
          ramp.reset();  // reset the ramp to 0
          ramp.setSlope(cart_vel_mag);
          cart_dir.setValue(rt_cv_cmd);  // set our cartesian direction to subscribed command
          current_cart_pos.setValue(
              arm_wam->tpoTpController.referenceInput.getValue());  // updating the current position to the actual low level commanded value
        }
        cart_vel_status = true;
        new_rt_cart_vel_cmd = false;
      } else {
        armHoldCurState();
      }
      break;

    case JOINT_VELOCITY:
      //Real-Time Joint Velocity Control Portion
      if (last_jnt_vel_msg_time + rt_msg_timeout > ros::Time::now())  // checking if a joint velocity message has been published and if it is within timeout
          {
        if (!jnt_vel_status) {
          jv_type jv_start;
          for (size_t i = 0; i < DOF; i++)
            jv_start[i] = 0.0;
          jv_track.setValue(jv_start);  // zeroing the joint velocity command
          arm_wam->trackReferenceSignal(jv_track.output);  // command the WAM to track the RT commanded up to (500 Hz) joint velocities
        } else if (new_rt_jnt_vel_cmd) {
          jv_track.setValue(rt_jv_cmd);  // set our joint velocity to subscribed command
        }
        jnt_vel_status = true;
        new_rt_jnt_vel_cmd = false;
      } else {
        armHoldCurState();
      }
      break;

    case ORIENTATION_VELOCITY:

      //Real-Time Angular Velocity Control Portion
      if (last_ortn_vel_msg_time + rt_msg_timeout > ros::Time::now())  // checking if a orientation velocity message has been published and if it is within timeout
          {
        if (!ortn_vel_status) {
          rpy_cmd.setValue(math::Vector<3>::type(0.0, 0.0, 0.0));  // zeroing the rpy command
          current_cart_pos.setValue(arm_wam->getToolPosition());  // Initializing the cartesian position
          current_rpy_ortn.setValue(toRPY(arm_wam->getToolOrientation()));  // Initializing the orientation

          systems::forceConnect(ramp.output, mult_angular.input1);  // connecting the ramp to multiplier
          systems::forceConnect(rpy_cmd.output, mult_angular.input2);  // connecting the rpy command to the multiplier
          systems::forceConnect(mult_angular.output, ortn_cmd_sum.getInput(0));  // adding the output of the multiplier
          systems::forceConnect(current_rpy_ortn.output,
                                ortn_cmd_sum.getInput(1));  // with the starting rpy orientation offset
          systems::forceConnect(ortn_cmd_sum.output, to_quat.input);
          systems::forceConnect(current_cart_pos.output,
                                rt_pose_cmd.getInput<0>());  // saving the original position to the pose.position
          systems::forceConnect(to_quat.output, rt_pose_cmd.getInput<1>());  // saving the summed and converted new quaternion commmand as the pose.orientation
          ramp.setSlope(ortn_vel_mag);  // setting the slope to the commanded magnitude
          ramp.stop();  // ramp is stopped on startup
          ramp.setOutput(0.0);  // ramp is re-zeroed on startup
          ramp.start();  // start the ramp
          arm_wam->trackReferenceSignal(rt_pose_cmd.output);  // command the WAM to track the RT commanded up to (500 Hz) cartesian velocity
        } else if (new_rt_ortn_vel_cmd) {
          ramp.reset();  // reset the ramp to 0
          ramp.setSlope(ortn_vel_mag);  // updating the commanded angular velocity magnitude
          rpy_cmd.setValue(rt_ortn_cmd);  // set our angular rpy command to subscribed command
          current_rpy_ortn.setValue(
              toRPY(arm_wam->tpoToController.referenceInput.getValue()));  // updating the current orientation to the actual low level commanded value
        }
        ortn_vel_status = true;
        new_rt_ortn_vel_cmd = false;
      } else {
        armHoldCurState();
      }
      break;

    case CARTESIAN_POSITION:
      //Real-Time Cartesian Position Control Portion
      if (last_cart_pos_msg_time + rt_msg_timeout > ros::Time::now())  // checking if a cartesian position message has been published and if it is within timeout
          {
        if (!cart_pos_status) {
          cp_track.setValue(arm_wam->getToolPosition());
          current_ortn.setValue(arm_wam->getToolOrientation());  // Initializing the orientation
          cp_rl.setLimit(rt_cp_rl);
          systems::forceConnect(cp_track.output, cp_rl.input);
          systems::forceConnect(cp_rl.output, rt_pose_cmd.getInput<0>());  // saving the rate limited cartesian position command to the pose.position
          systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>());  // saving the original orientation to the pose.orientation
          arm_wam->trackReferenceSignal(rt_pose_cmd.output);  //Commanding the WAM to track the real-time pose command.
        } else if (new_rt_cart_pos_cmd) {
          cp_track.setValue(rt_cp_cmd);  // Set our cartesian positions to subscribed command
          cp_rl.setLimit(rt_cp_rl);  // Updating the rate limit to subscribed rate to control the rate of the moves
        }
        cart_pos_status = true;
        new_rt_cart_pos_cmd = false;
      } else {
        armHoldCurState();
      }
      break;

    /*case JOINT_POSITION:
      //Real-Time Joint Position Control Portion
      if (last_jnt_pos_msg_time + rt_msg_timeout > ros::Time::now())  // checking if a joint position message has been published and if it is within timeout
          {
        if (!jnt_pos_status) {
          jp_type jp_start = arm_wam->getJointPositions();
          jp_track.setValue(jp_start);  // setting initial the joint position command
          jp_rl.setLimit(rt_jp_rl);
          systems::forceConnect(jp_track.output, jp_rl.input);
          arm_wam->trackReferenceSignal(jp_rl.output);  // command the WAM to track the RT commanded up to (500 Hz) joint positions
        } else if (new_rt_jnt_pos_cmd) {
          jp_track.setValue(rt_jp_cmd);  // set our joint position to subscribed command
          jp_rl.setLimit(rt_jp_rl);  // set our rate limit to subscribed rate to control the rate of the moves
        }
        jnt_pos_status = true;
        new_rt_jnt_pos_cmd = false;
      } else {
        armHoldCurState();
      }
      break;*/

    case JOINT_POSITION:

        if (!jnt_pos_status) {
          jp_type jp_start = arm_wam->getJointPositions();
          jp_track.setValue(jp_start);  // setting initial the joint position command
          jp_rl.setLimit(rt_jp_rl);
          systems::forceConnect(jp_track.output, jp_rl.input);
          arm_wam->trackReferenceSignal(jp_rl.output);  // command the WAM to track the RT commanded up to (500 Hz) joint positions
          jnt_pos_status = true;
        } else {
          jp_track.setValue(rt_jp_cmd); // set our joint position to subscribed command
        }

      break;

    case JOINT_TORQUE:
      // Real-Time Joint Torque Control Portion
      if (last_jnt_trq_msg_time + rt_msg_timeout > ros::Time::now())  // Checking if a joint torque message has been published and if it is within timeout
          {
        if (!jnt_trq_status) {
          jt_type jt_start = arm_wam->getJointTorques();
          jt_track.setValue(jt_start);
          arm_wam->trackReferenceSignal(jt_track.output);
        } else if (new_rt_jnt_trq_cmd) {
          jt_track.setValue(rt_jt_cmd);
        }
        jnt_trq_status = true;
        new_rt_jnt_trq_cmd = false;
      } else {
        armHoldCurState();
      }
      break;
    default:
      armHoldCurState();
  }
}

/*
 * Initialize the subscribers and publishers. Initialize the messages with their default values
 */
template<size_t DOF>
void BarrettArmInterface<DOF>::init() {
  cart_vel_status = false;  //Bool for determining cartesian velocity real-time state
  ortn_vel_status = false;  //Bool for determining orientation velocity real-time state
  new_rt_cart_vel_cmd = new_rt_jnt_vel_cmd = new_rt_ortn_vel_cmd =
      new_rt_jnt_pos_cmd = new_rt_cart_pos_cmd = new_rt_jnt_trq_cmd = false;  //Bool for determining if a new real-time message was received
  arm_pm->getExecutionManager()->startManaging(ramp);  //starting ramp manager

  // Gains Subscribers
  arm_jnt_pos_gns_sub = nh->subscribe(
      ARM_UPDATE_JNTPOSGAINS_TOPIC, 1,
      &BarrettArmInterface<DOF>::armUpdateJntPosGains, this);
  arm_jnt_vel_gns_sub = nh->subscribe(
      ARM_UPDATE_JNTVELGAINS_TOPIC, 1,
      &BarrettArmInterface<DOF>::armUpdateJntVelGains, this);
  arm_cart_pos_gns_sub = nh->subscribe(
      ARM_UPDATE_CARTPOSGAINS_TOPIC, 1,
      &BarrettArmInterface<DOF>::armUpdateCartPosGains, this);
  arm_cart_pose_gns_sub = nh->subscribe(
      ARM_UPDATE_CARTPOSEGAINS_TOPIC, 1,
      &BarrettArmInterface<DOF>::armUpdateCartPoseGains, this);
  arm_cart_vel_cmd_sub = nh->subscribe(
      ARM_UPDATE_CARTVEL_TOPIC, 1, &BarrettArmInterface<DOF>::armUpdateCartVel,
      this);
  arm_jnt_vel_cmd_sub = nh->subscribe(
      ARM_UPDATE_JNTVEL_TOPIC, 1, &BarrettArmInterface<DOF>::armUpdateJntVel,
      this);
  arm_ortn_vel_cmd_sub = nh->subscribe(
      ARM_UPDATE_ORTNVEL_TOPIC, 1, &BarrettArmInterface<DOF>::armUpdateOrtnVel,
      this);
  arm_cart_pos_cmd_sub = nh->subscribe(
      ARM_UPDATE_CARTPOS_TOPIC, 1, &BarrettArmInterface<DOF>::armUpdateCartPos,
      this);
  //arm_jnt_pos_cmd_sub = nh->subscribe(
   //   ARM_UPDATE_JNTPOS_TOPIC, 1, &BarrettArmInterface<DOF>::armUpdateJntPos,
   //   this);

  // Joint Publisher
  arm_js_pub = nh->advertise<sensor_msgs::JointState>(ARM_JS_TOPIC, 1);

  // Cartesian Publisher
  arm_es_pub = nh->advertise<wam_msgs::EndpointState>(ARM_ES_TOPIC, 1);

  // Advertise the following services
  arm_toggle_gravity_srv = nh->advertiseService(
      ARM_TOGGLE_GRAV, &BarrettArmInterface<DOF>::armToggleGravity, this);
  arm_update_gravity_srv = nh->advertiseService(
      ARM_UPDATE_GRAV, &BarrettArmInterface<DOF>::armUpdateGravity, this);
  arm_hold_jntpos_srv = nh->advertiseService(
      ARM_HOLD_JNTPOS_TOPIC, &BarrettArmInterface<DOF>::armHoldJointPos, this);
  arm_hold_cartpos_srv = nh->advertiseService(
      ARM_HOLD_CARTPOS_TOPIC, &BarrettArmInterface<DOF>::armHoldCartPos, this);
  arm_hold_cartpose_srv = nh->advertiseService(
      ARM_HOLD_CARTPOSE_TOPIC, &BarrettArmInterface<DOF>::armHoldCartPose,
      this);
  arm_mv_home_srv = nh->advertiseService(ARM_MOVE_HOME_TOPIC,
                                         &BarrettArmInterface<DOF>::armMoveHome,
                                         this);
  arm_jnt_mv_to_srv = nh->advertiseService(
      ARM_JOINT_MOVETO_TOPIC, &BarrettArmInterface<DOF>::armJointMoveTo, this);
  arm_idle_srv = nh->advertiseService(ARM_IDLE_TOPIC,
                                      &BarrettArmInterface<DOF>::armIdle, this);
  arm_get_DOF_srv = nh->advertiseService(ARM_GET_DOF_TOPIC,
                                         &BarrettArmInterface<DOF>::armGetDOF,
                                         this);
  arm_get_safety_srv = nh->advertiseService(
      ARM_SAFETY_STATE_TOPIC, &BarrettArmInterface<DOF>::armGetSafetyState,
      this);
  arm_set_move_accel_srv = nh->advertiseService(
      ARM_SET_MOVE_ACCEL_TOPIC, &BarrettArmInterface<DOF>::armSetMoveAccel,
      this);
  arm_set_move_vel_srv = nh->advertiseService(
      ARM_SET_MOVE_VEL_TOPIC, &BarrettArmInterface<DOF>::armSetMoveVel, this);
  arm_apply_vel_accel_srv = nh->advertiseService(
      ARM_APPLY_ACCEL_VEL_TOPIC, &BarrettArmInterface<DOF>::armApplyVelAccel,
      this);
  arm_select_rt_mode_srv = nh->advertiseService(
      ARM_SELECT_RT_MODE_TOPIC, &BarrettArmInterface<DOF>::armSelectRTMode,
      this);
  // Initialize the Joint state publisher message with its default values
  js_info.name.resize(DOF);
  arm_pm->getSafetyModule();
  for (size_t i = 0; i < DOF; ++i) {
    js_info.name[i] = jnt_names[i];
  }
  // Turn on the gravity compensation by default
  arm_wam->gravityCompensate();
}
}  // namespace
#endif  // INCLUDE_BARRETT_ARM_H_
