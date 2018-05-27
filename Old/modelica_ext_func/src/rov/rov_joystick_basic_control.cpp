#include <ros/ros.h>
#include "modelica_ext_func/ModROV.h"
#include <sensor_msgs/Joy.h>

ros::Publisher pub;

int scale;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  modelica_ext_func::ModROV rovVal;
  rovVal.rVal[0] = scale*joy->axes[7];
  rovVal.rVal[1] = scale*joy->axes[7];
  rovVal.rVal[2] = scale*joy->axes[7];
  rovVal.rVal[3] = scale*joy->axes[7];
  rovVal.rVal[4] = scale*joy->axes[4];
  rovVal.rVal[5] = scale*joy->axes[4];
  pub.publish(rovVal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rov_joystick_basic_control");
    ros::NodeHandle n;
    n.param<int>("scale", scale, 1);
    
    pub = n.advertise<modelica_ext_func::ModROV>("control_values", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();

    return 0;
}