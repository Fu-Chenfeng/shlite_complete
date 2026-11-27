#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace shlite_custom_rviz
{
class GoalDisplay : public moveit_rviz_plugin::MotionPlanningDisplay
{
public:
  void onInitialize() override
  {
    moveit_rviz_plugin::MotionPlanningDisplay::onInitialize();
    publisher_ = node_.advertise<moveit_msgs::RobotState>("/custom_trajectory/rviz_goal", 1);
  }

  void update(float wall_dt, float ros_dt) override
  {
    moveit_rviz_plugin::MotionPlanningDisplay::update(wall_dt, ros_dt);
    elapsed_ += wall_dt;
    if (elapsed_ < 0.1f || !publisher_ || publisher_.getNumSubscribers() == 0 || !getQueryGoalStateHandler())
      return;
    elapsed_ = 0;
    const auto state = getQueryGoalState();
    if (state)
    {
      moveit_msgs::RobotState message;
      moveit::core::robotStateToRobotStateMsg(*state, message);
      publisher_.publish(message);
    }
  }

private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  float elapsed_ = 0;
};
}
PLUGINLIB_EXPORT_CLASS(shlite_custom_rviz::GoalDisplay, rviz::Display)
