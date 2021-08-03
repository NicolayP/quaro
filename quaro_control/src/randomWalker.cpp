#include <quaro_control/randomWalker.hpp>
#include <cstdlib>

namespace random_walker{
  RandomWalker::RandomWalker(){}
  RandomWalker::~RandomWalker(){}

  bool RandomWalker::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& cn){
    effortJointHandles.clear();
    std::vector<std::string> jointNames;

    if(!cn.getParam("joints", jointNames)){
      ROS_ERROR("No Joints in given namespace.");
    }
    if(!hw){
      ROS_ERROR("Effort Joint Interface is empty in hardware Interface.");
      return false;
    }
    for(unsigned int i=0; i<jointNames.size(); i++){
      effortJointHandles.push_back(hw->getHandle(jointNames[i]));
    }
    buffer_command_effort.resize(effortJointHandles.size(), 0.0);
    buffer_current_efforts.resize(effortJointHandles.size(), 0.0);
    buffer_current_positions.resize(effortJointHandles.size(), 0.0);
    buffer_current_velocities.resize(effortJointHandles.size(), 0.0);

    return true;
  }

  void RandomWalker::starting(const ros::Time& time){
    for(unsigned int i=0; i<effortJointHandles.size(); i++){
      buffer_current_positions[i] = effortJointHandles[i].getPosition();
      buffer_current_velocities[i] = effortJointHandles[i].getVelocity();
      buffer_current_efforts[i] = effortJointHandles[i].getEffort();
    }
  }

  void RandomWalker::update(const ros::Time& time, const ros::Duration& period){
    for(unsigned int i=0; i < effortJointHandles.size(); i++){
      buffer_current_positions[i] = effortJointHandles[i].getPosition();
      buffer_current_velocities[i] = effortJointHandles[i].getVelocity();
      buffer_current_efforts[i] = effortJointHandles[i].getEffort();
      int walk = std::rand()%3;
      if (walk == 0){
        buffer_command_effort[i] = buffer_command_effort[i] - 1;
      }else if(walk == 1){
        buffer_command_effort[i] = buffer_command_effort[i] + 1;
      }else{
        buffer_command_effort[i] = buffer_command_effort[i];
      }
      effortJointHandles[i].setCommand(buffer_command_effort[i]);

    }
  }

  void RandomWalker::stopping(const ros::Time& time){}
}

PLUGINLIB_EXPORT_CLASS(random_walker::RandomWalker, controller_interface::ControllerBase)
