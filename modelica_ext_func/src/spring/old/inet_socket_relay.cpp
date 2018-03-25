#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

double rec = 0.0;

void storeCallback(const std_msgs::StringConstPtr& str)
{
    rec = atof(str->data.c_str());
    // printf("Here is the ros val: %s \n", str.data.c_str());
    printf("Here it is with the arrow -> %s \n", str->data.c_str());
    // printf("Here it is with just a dot. %s \n", str.data);
    printf("Here is the stored value: %lf \n", rec);
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inet_socket_relay");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("model_values", 1);
    std_msgs::String val;
    // std::stringstream ss;

    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);

    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    double buf = 0;
    struct sockaddr_in serv_addr, cli_addr;
    int errorCheck;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = 9090;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR on binding");
    }
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    while(ros::ok())
    {
        std::stringstream ss;
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) {
            error("ERROR on accept");
        }

        bzero(buffer,256); buf = 0;

        errorCheck = read(newsockfd,buffer,255);
        if (errorCheck < 0) {
            error("ERROR reading from socket");
        }
        printf("Here is the message: %s\n",buffer);

        // buf = 15 * (2 - atof(buffer));
        buf = atof(buffer);

        printf("Here is the ingoing value: %lf \n", rec);
        sprintf(buffer, "%lf", rec);
        printf("Here is the ingoing message: %s \n", buffer);
        errorCheck = write(newsockfd, buffer, strlen(buffer));
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }

        close(newsockfd);

        ss << buf;
        printf("Here is the stream value: %s \n", ss.str().c_str());
        val.data=ss.str();
        printf("Here is the outgoing value: %s \n", val.data.c_str());
        // printf("Here it is again with an arrow -> %s \n", val->data.c_str());
        // printf("Here it is with just a dot. %s \n", val.data);
        pub.publish(val);
        ros::spinOnce();
    }
    close(sockfd);
    ros::shutdown();

    return 0;
}