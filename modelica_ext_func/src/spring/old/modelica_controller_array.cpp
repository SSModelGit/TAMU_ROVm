#include "ros/ros.h"
#include "modelica_ext_func/Num.h"

ros::Publisher pub;

void controlCallBack(const modelica_ext_func::Num::ConstPtr& inVal) {

    // find control values
    modelica_ext_func::Num outVal;
    outVal.val_1 = 5 * (2 - inVal->val_1);
    outVal.val_2 = 5 * (2 - inVal->val_2);
    
    printf("Incoming Values: %lf | %lf \n", inVal->val_1, inVal->val_2);
    printf("Outgoing Values: %lf | %lf \n\n", outVal.val_1, outVal.val_2);

    pub.publish(outVal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "modelica_controller_array");
    ros::NodeHandle n;
    
    pub = n.advertise<modelica_ext_func::Num>("control_values", 1);
    ros::Subscriber sub = n.subscribe("model_values", 1, controlCallBack);

    ros::spin();

    return 0;
}