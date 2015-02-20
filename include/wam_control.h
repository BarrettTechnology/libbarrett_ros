#ifndef WAM_CONTROL_H_
#define WAM_CONTROL_H_

#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

template<size_t DOF>
class BarrettBot : public hardware_interface::RobotHW
{
public:
 BarrettBot()
 {
   std::vector<hardware_interface::JointStateHandle> state_handle;
   std::vector<hardware_interface::JointHandle> pos_handle;
   std::vector<std::string> joint_names = {"w1", "w2", "w3", "w4", "w5", "w6", "w7"};

   state_handle.reserve(DOF);
   pos_handle.reserve(DOF);

   for(size_t i = 0; i < DOF; ++i){
     // connect and register the joint state interface
     state_handle.push_back(hardware_interface::JointStateHandle(joint_names[i], &pos[i], &vel[i], &eff[i]));
     jnt_state_interface.registerHandle(state_handle[i]);

     // connect and register the joint position interface
     pos_handle.push_back(hardware_interface::JointHandle(jnt_state_interface.getHandle(joint_names[i]), &pos_cmd[i]));
     jnt_pos_interface.registerHandle(pos_handle[i]);
   }
   registerInterface(&jnt_state_interface);
   registerInterface(&jnt_pos_interface);
  }
 virtual ~BarrettBot(){}
 double pos_cmd[DOF];
 double pos[DOF];
 double vel[DOF];
 double eff[DOF];
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

};
#endif // WAM_CONTROL_H
