#include <ros/ros.h>
#include "modelica_ext_func/ModROSspring.h"
#include <sensor_msgs/Joy.h>

ros::Publisher pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  modelica_ext_func::ModROSspring springVal;
  springVal.sVal[0] = 10*joy->axes[1];
  springVal.sVal[1] = 5*joy->axes[4];
  pub.publish(springVal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_mod");
    ros::NodeHandle n;
    
    pub = n.advertise<modelica_ext_func::ModROSspring>("mod_ros_joy", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();

    return 0;
}