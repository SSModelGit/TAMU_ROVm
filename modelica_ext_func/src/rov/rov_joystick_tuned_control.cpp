#include <ros/ros.h>
#include "modelica_ext_func/ModROV.h"
#include <sensor_msgs/Joy.h>
#include <stdio.h>

ros::Publisher pub;

const double inverseControlMat[6][6]= {
    {0.388909, -0.0, -0.319228, 0.343256, 0.102977, -0.353553},
    {0.388909, 0.0, 0.319228, -0.343256, -0.102977, -0.353553},
    {0.318198, -0.0, 0.387879, 0.343256, 0.102977, 0.353553},
    {0.318198, 0.0, -0.387879, -0.343256, -0.102977, 0.353553},
    {-0.0, 0.5, 0.00485437, 0.0485437, -0.485437, 0.0},
    {0.0, 0.5, -0.00485437, -0.0485437, 0.485437, -0.0}
};

const int invCMatDim = 6;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    modelica_ext_func::ModROV rovVal;
    rovVal.rVal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float scale = 2.0;
    
    double joyVal[8] = {joy->axes[1], joy->axes[0], joy->axes[4], ((-0.5 * joy->axes[2]) + 0.5), ((-0.5 * joy->axes[5]) + 0.5), joy->axes[3], joy->axes[6], joy->axes[7]};
    // joystick mappings - http://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux

    /*
    personal mappings:
    joyVal[0] => Forward movement => axes[1]
    joyVal[1] => Lateral movement => axes[0]
    joyVal[2] => Vertical movement => axes[4]
    joyVal[3] => Roll => axes[2]
    joyVal[4] => Pitch => axes[5]
    joyVal[5] => Yaw => axes[3]
    joyVal[6] => Not set
    joyVal[7] => Not set 
    */

    // set the values of the vertical props
    
    for(int s = 0; s < sizeof(joyVal); s++) {
        joyVal[s] *= scale; // scale up joystick values
    }
    
    printf("_  _    _                 _  _  _\n");
    // Multiply inverse matrix by 'desired' joystick values
    for(int i = 0; i < invCMatDim; i++) {
        for(int j = 0; j < invCMatDim; j++) {
            rovVal.rVal[i] += inverseControlMat[i][j] * joyVal[j];
        }
        printf("| %lf |    | %lf %lf %lf %lf %lf %lf | | %lf |\n", rovVal.rVal[i], inverseControlMat[i][0], inverseControlMat[i][1], inverseControlMat[i][2], 
        inverseControlMat[i][3], inverseControlMat[i][4], inverseControlMat[i][5], joyVal[i]);
    }
    printf("_  _    _                 _  _  _\n");
    
    pub.publish(rovVal); // publish joystick-based control values
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rov_joystick_tuned_control_relay");
    ros::NodeHandle n;
    
    pub = n.advertise<modelica_ext_func::ModROV>("control_values", 1);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();

    return 0;
}