// A working client call function, connects to a TCP socket, on localhost, port number specified as a parameter in the function call
// Sends the value recieved from Modelica to the server side (located on ROS), and recieves back a new value, which is sent to Modelica
// The server's current purpose is to recieve the value from Modelica, perform some control function, and return the new value
// It is located in the comments below the ROSClientCall code

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

double ROSClientCall(double time, int portno, double que)
{
    printf("time: %f\n", time); // time keeper

    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    double buf;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

    server = gethostbyname("localhost");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    sprintf(buffer, "%1f", que);
    n = write(sockfd, buffer, strlen(buffer));
    if (n < 0) 
         error("ERROR writing to socket");

    n = read(sockfd, buffer, 255);
    buf = atof(buffer);
    if (n < 0) 
         error("ERROR reading from socket");
    printf("%1f\n",buf);

    close(sockfd);

    return buf;
}

/*
Server code (located on ros)

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

*/
