#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// #include <geometry_msgs/Twist.h>


class LaserScan
{
public:
  LaserScan();
  // geometry_msgs::Twist vel;

private:
  void lasercall(const sensor_msgs::LaserScan::ConstPtr& laser);

  ros::NodeHandle nh_laser_;

  float min_angle_, max_angle_;
  float dist;
  ros::Subscriber laser_sub_;
};

LaserScan::LaserScan():
  min_angle_(0.0),
  max_angle_(0.0)
{
  laser_sub_ = nh_laser_.subscribe<sensor_msgs::LaserScan>("/scan", 50, &LaserScan::lasercall, this);
}

void LaserScan::lasercall(const sensor_msgs::LaserScan::ConstPtr& laser)
{
  ROS_INFO("INSIDE THE LASERSCAN FUNCTION");

  //laser->angle_min = min_angle_;
  //laser->angle_max = max_angle_;
  
  dist = laser->ranges[0];
  ros::Rate r(1.0);

  ROS_INFO("The min angle is: %f and the max angle is: %f", laser->angle_min, laser->angle_max);
  ROS_INFO("The distance value is: %f", dist);

  if(dist == 0.25)
    {
      ROS_INFO("LETS DANCE BABY");
      ros::Duration(2.0).sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dist_scan");
  LaserScan dist_scan;

  ROS_INFO("IN THE MAIN");

  ros::spin();
}


/*
int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}
*/
