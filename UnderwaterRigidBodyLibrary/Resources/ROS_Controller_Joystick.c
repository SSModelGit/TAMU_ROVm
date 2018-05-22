#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <assert.h>

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

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void ROS_Controller_Joystick(double time, int portno, double *res)
{
    printf("time: %f\n", time); // time keeper

    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256]; // initializes character array buffer for socket transmission

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

    // Interact with socket (read and write):
    
    printf("Requesting input");
    n = write(sockfd, "Request", strlen("Request"));
    if (n < 0) 
        error("ERROR writing to socket");

    n = read(sockfd, buffer, 255);
    if (n < 0) 
        error("ERROR reading from socket");
    splitter(res, buffer, ",");
    printf("q1: %lf | q2: %lf | q3: %lf | q4: %lf | q5: %lf | q6: %lf \n\n", res[0], res[1], res[2], res[3], res[4], res[5]);

    close(sockfd); // close socket
}
