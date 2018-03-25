#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inet_socket");

    ros::NodeHandle n;
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
        buf = 15 * (2 - atof(buffer));
        sprintf(buffer, "%lf", buf);
        errorCheck = write(newsockfd, buffer, strlen(buffer));
        if (errorCheck < 0) {
            error("ERROR writing to socket");
        }
        close(newsockfd);
    }
    close(sockfd);
    ros::shutdown();

    return 0;
}