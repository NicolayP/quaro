#ifndef RANDOM_WALKER_HPP
#define RANDOM_WALKER_HPP

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

namespace random_walker
{
   class RandomWalker : public controller_interface::Controller<hardware_interface::EffortJointInterface>
   {
   public:
        RandomWalker();
        virtual ~RandomWalker();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& cn);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);

   private:
        std::vector<hardware_interface::JointHandle> effortJointHandles;
        std::vector<double> buffer_command_effort;
        std::vector<double> buffer_current_positions;
        std::vector<double> buffer_current_velocities;
        std::vector<double> buffer_current_efforts;
   };
}
#endif
