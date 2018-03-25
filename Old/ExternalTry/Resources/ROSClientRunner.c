// Works as a debugger, calling the specified external function to check its function
// ROSClientDo is the output file after compilation
// Currently linked to ROSClientCall_3.c

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

double ROSClientCall(double time, int portno, double que);

void main(int argc, char *argv[]) {
    printf("%s\n", argv[1]);
    printf("%s\n", argv[2]);
    int port = atoi(argv[1]);
    double query = atof(argv[2]);
    printf("%d\n", port);
    printf("%lf\n", query);
    printf("%lf\n", ROSClientCall(1.0, port, query));
}