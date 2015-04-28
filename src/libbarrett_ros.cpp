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
#include <urdf/model.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <boost/thread.hpp>
#include <barrett/systems/real_time_execution_manager.h>

//#include <barrett_arm.h>
//#include <barrett_hand.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

// Ros param names
const std::string robot_description = "robot_description";
const std::string tip_joint = "tip_joint";
using namespace barrett;

template<size_t DOF>
class BarrettHW : public hardware_interface::RobotHW, public systems::SingleIO<
    typename units::JointPositions<DOF>::type,
    typename units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
  BarrettHW(systems::Wam<DOF> *wam, ProductManager *pm, ros::NodeHandle &nh, const std::string& sysName = "BarrettHW")
  :systems::SingleIO<jp_type, jt_type>(sysName),arm_wam(wam), arm_pm(pm), jnt_pos(0.0),
  jnt_vel (0.0), jnt_trq(0.0), jnt_cmd(0.0), cm(this), n(nh)
  {
    //const char* j_n[] = {"proficio_joint_1", "proficio_joint_2", "proficio_joint_3",
    //   "j4", "j5", "j6", "j7"};
    //std::vector<std::string> jnt_names(j_n, j_n+7);
    this->parseURDF();
    std::vector<hardware_interface::JointStateHandle> state_handle;
    std::vector<hardware_interface::JointHandle> pos_handle;
    state_handle.reserve(DOF);
    pos_handle.reserve(DOF);

    // Create a handle for Joint states and register them
    for(size_t i = 0; i < DOF; ++i) {
      state_handle.push_back(hardware_interface::JointStateHandle(
              jnt_names[i], &pos[i], &vel[i], &eff[i]));
      jnt_state_interface.registerHandle(state_handle[i]);
    }

    registerInterface(&jnt_state_interface);

    // Create a handle for the Joint commands and register them
    for(size_t i = 0;i < DOF; ++i) {
      pos_handle.push_back(hardware_interface::JointHandle(
              jnt_state_interface.getHandle(jnt_names[i]), &cmd[i]));
      jnt_eff_interface.registerHandle(pos_handle[i]);
    }

    registerInterface(&jnt_eff_interface);
  }

protected:

  /*
   * Parse the URDF and get the Joint names from it
   */
  void parseURDF() {
    urdf::Model urdf;
    std::string urdf_data, tip_joint_name;

    //Read from the param server into the variables and initialize the URDF string
    n.getParam(robot_description, urdf_data);
    n.getParam(tip_joint, tip_joint_name);
    if (!urdf.initString(urdf_data)) {
      ROS_ERROR("Failed to parse urdf file");
    }

    //Get the Joint handle of the tip joint by passing its name
    boost::shared_ptr<const urdf::Joint> joint = urdf.getJoint(tip_joint_name);

    jnt_names.resize(DOF);

    for(size_t i = DOF; i > 0; --i) {
      // Keep searching till a new revolute joint is found
      while(std::find(jnt_names.begin(), jnt_names.end(), joint->name)
          != jnt_names.end() || joint->type != urdf::Joint::REVOLUTE) {
        // Get the next Joint
        joint = urdf.getLink(joint->parent_link_name)->parent_joint;
        if(!joint.get()) {
          ROS_ERROR("Number of joints specified in the URDF does not match with the hardware being used");
          throw std::runtime_error("Ran out of Joints");
        }
      }
      jnt_names[i-1] = joint->name;
    }

  }
  /*
   * Read the Joint state of the robot from the hardware
   */
  void read() {
    jnt_pos = this->input.getValue();
    jnt_trq = arm_wam->getJointTorques();
    jnt_vel = arm_wam->getJointVelocities();

    // Read from barrett units into the registered joint states
    for(size_t i = 0; i < DOF; ++i) {
      pos[i] = jnt_pos [i];
      vel[i] = jnt_vel[i];
      eff[i] = jnt_trq[i];
    }
  }
  /*
   * Update the controller manager
   */
  void update() {
    cm.update(static_cast<ros::Time>(highResolutionSystemTime()), static_cast<ros::Duration>(arm_pm->getExecutionManager()->getPeriod()));
  }

  /*
   * Write the Joint commands into the hardware
   */
  void write() {
    // Copy the Joint commands into barrett units
    for(size_t i = 0; i < DOF; ++i)
    jnt_cmd[i] = cmd[i];
    // Write the joint command into the output of this system
    this->outputValue->setData(&jnt_cmd);

  }

  /*
   * The control loop running at 500 Hz
   */
  virtual void operate() {
    this->read();
    this->update();
    this->write();
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  double cmd[DOF];
  double pos[DOF];
  double vel[DOF];
  double eff[DOF];
  std::vector<std::string> jnt_names;
  systems::Wam<DOF> *arm_wam;
  ProductManager *arm_pm;
  jp_type jnt_pos;
  jv_type jnt_vel;
  jt_type jnt_trq, jnt_cmd;

  controller_manager::ControllerManager cm;
  ros::NodeHandle n;

  DISALLOW_COPY_AND_ASSIGN(BarrettHW);
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
             systems::Wam<DOF>& wam) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  ros::init(argc, argv, "libbarrett_ros");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  wam.gravityCompensate();
  BarrettHW<DOF> bhw(&wam, &pm, nh);
  systems::connect(wam.jpOutput, bhw.input);
  wam.trackReferenceSignal(bhw.output);
  // Wait for the user to press Shift-idle
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

  return 0;
}
