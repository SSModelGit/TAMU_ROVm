#include <ros/ros.h>
#include <modelica_ext_func/Num.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

void commaconcatanater(char *s, double *dat)
{
    s[0] = '\0';
	char s1[100];
	for (int i = 0; i< sizeof(dat); i++)
	{
		sprintf(s1,"%f,",dat[i]);
		strcat(s,s1);
	}
}

void splitter(double *outVal, char *s, char *delim) {
    // http://www.rosipov.com/blog/c-strtok-usage-example/
    int j, i = 0;
    char *token[80];

    token[0] = strtok(s, delim);
    while (token[i] != NULL) {
        i++;
        token[i] = strtok(NULL, delim);
    }

    for (j=0; j<=i-1; j++) {
        outVal[j] = atof(token[j]);
    }
}

double rec[2] = {0.0, 0.0};

void storeCallback(const modelica_ext_func::Num::ConstPtr& inVal)
{
    rec[0] = inVal->val_1;
    rec[1] = inVal->val_2;
    printf("Incoming values: %lf | %lf \n", rec[0], rec[1]);
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inet_socket_relay_array");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<modelica_ext_func::Num>("model_values", 1);
    modelica_ext_func::Num val;

    ros::Subscriber sub = nh.subscribe("control_values", 1, storeCallback);

    int sockfd, newsockfd, portno;
    socklen_t clilen;

    char buffer[256];
    double sockBuf[2]; // buffer array for interaction with socket
    double rosBuf[2]; // buffer array for transmission via ROS

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
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) {
            error("ERROR on accept");
        }

        errorCheck = read(newsockfd,buffer,255);
        if (errorCheck < 0) {
            error("ERROR reading from socket");
        }
        printf("Outgoing socket message: %s\n",buffer);

        // buf = 15 * (2 - atof(buffer));
        splitter(rosBuf, buffer, ",");

        memcpy(sockBuf, rec, sizeof(rec));

        printf("To socket values: %lf | %lf\n", sockBuf[0], sockBuf[1]);
        commaconcatanater(buffer, sockBuf);
        printf("Ingoing socket message: %s \n\n", buffer);
        errorCheck = write(newsockfd, buffer, strlen(buffer));
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }

        close(newsockfd);

        val.val_1 = rosBuf[0];
        val.val_2 = rosBuf[1];
        printf("Outgoing values: %lf | %lf \n\n", val.val_1, val.val_2);
        pub.publish(val);
        ros::spinOnce();
    }
    close(sockfd);
    ros::shutdown();

    return 0;
}