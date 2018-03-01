// Failed attempt at connecting ROS with Modelica via pipes - included for reference of previous attempts
// No error recieved during compilation, but during simulation, the entire simulation window of SystemModeler would either freeze,
//   or the simulation would proceed extremely slowly (around 5 hours taken for 5 seconds of simulation)
// Cause behind problem not detected, but the approach was not pursued

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAX_BUF 1024

double ROSClientCall()
{
    int fd;
    char * myfifo = "/tmp/myfifo";
    int buf = 1;
    printf("Debug 1. Current: %d | ", buf);

    /* open, read, and display the message from the FIFO */
    fd = open(myfifo, O_RDONLY);
    printf("Opened. | ");
    read(fd, &buf, sizeof(buf));
    printf("Read : %d | ", buf);
    close(fd);
    printf("Closed. \n");
    return (double) buf;
}