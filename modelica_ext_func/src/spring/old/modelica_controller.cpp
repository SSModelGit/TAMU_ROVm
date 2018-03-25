#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

ros::Publisher pub;

void controlCallBack(const std_msgs::StringConstPtr& str) {
    // find control value
    double buf = 5 * (2 - atof(str->data.c_str()));
    // printf("Here is the ros val: %s \n", str.data.c_str());
    printf("Here it is with the arrow -> %s \n", str->data.c_str());
    // printf("Here it is with just a dot. %s \n", str.data);
    printf("Here is the stored value: %lf \n", buf);

    // publish as std_msgs::String
    std_msgs::String val;
    std::stringstream ss;
    ss << buf;
    printf("Here is the stream value: %s \n", ss.str().c_str());
    val.data=ss.str();
    printf("Here is the outgoing value: %s \n", val.data.c_str());
    // printf("Here it is again with an arrow -> %s \n", val->data.c_str());
    // printf("Here it is with just a dot. %s \n", val.data);
    pub.publish(val);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "modelica_controller");
    ros::NodeHandle n;
    
    pub = n.advertise<std_msgs::String>("control_values", 1);
    ros::Subscriber sub = n.subscribe("model_values", 1, controlCallBack);

    ros::spin();

    return 0;
}