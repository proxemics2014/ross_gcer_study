#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>



class TeleopTurtle
{
public:
  TeleopTurtle();
  geometry_msgs::Twist vel;


private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber wii_sub_;
  
};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 100, &TeleopTurtle::joyCallback, this);
  wii_sub_ = nh_.subscribe<sensor_msgs::Joy>("/wiimote/nunchuk", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("Entering joyCallback");
 /* vel.angular.z = 0;
  vel.linear.x = 0;
  vel_pub_.publish(vel); */
  if(joy->buttons[10] != 1)
  {


      vel.linear.x = l_scale_*joy->axes[linear_];
      vel_pub_.publish(vel);
//      wii_sub_ = nh_.subscribe<sensor_msgs::Joy>("/wiimote/nunchuk", 10, &TeleopTurtle::joyCallback, this);
  }
  else
  {
      if (joy->buttons[4] == 1)
      {
          vel.linear.x = 0.2*joy->buttons[4];
          ROS_INFO("You are going UP with Linear: %f, Angular: %f", vel.linear.x, vel.angular.z);
          //joy->buttons[4] = 0;
      }
      else if (joy->buttons[5] == 1)
      {
          vel.angular.z = -0.4*joy->buttons[5];
          ROS_INFO("You are going RIGHT with Linear: %f, Angular: %f", vel.linear.x, vel.angular.z);
          //joy->buttons[5] = 0;
      }
      else if (joy->buttons[6] == 1)
      {
          vel.linear.x = -0.2*joy->buttons[6];
          ROS_INFO("You are going DOWN with Linear: %f, Angular: %f", vel.linear.x, vel.angular.z);
          //joy->buttons[6] = 0;
      }
      else if (joy->buttons[7] == 1)
      {
          vel.angular.z = 0.4*joy->buttons[7];
          ROS_INFO("You are going LEFT with Linear: %f, Angular: %f", vel.linear.x, vel.angular.z);
          //joy->buttons[7] = 0;

      }
      else if (joy->buttons[8] == 1 && joy->buttons[9] == 1)
      {
          vel.linear.x = 0*joy->buttons[8];
          vel.angular.z = 0*joy->buttons[8];
          ROS_INFO("You have stopped with Linear: %f, Angular: %f", vel.linear.x, vel.angular.z);
      }
  }
  ROS_INFO("STOPPED: %d,  %d, %d, %d", joy->buttons[4], joy->buttons[5], joy->buttons[6], joy->buttons[7]);
  //vel.linear.x = 0;
  //vel.angular.z = 0;
  vel_pub_.publish(vel); 
  ROS_INFO("Linear_Vel = %f, Angular_Vel = %f", vel.linear.x, vel.linear.z);
  ROS_INFO(" ");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_teleop");
  TeleopTurtle teleop_turtle;

  ROS_INFO("In the Main");

  ros::spin();
}
