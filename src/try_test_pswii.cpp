#include <ross_gcer_study/TeleopTurtle.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_teleop");
  TeleopTurtle teleop_turtle;

  ROS_INFO("In the Main");

  ros::spin();
}
