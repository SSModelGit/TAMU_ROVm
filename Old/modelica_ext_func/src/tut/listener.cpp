#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

int loopChanger = 0;
char * myfifo = "/tmp/myfifo";

void handler(int sig) {
  /* remove the FIFO */
  unlink(myfifo);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    int fd;
    int kingkong = atoi(msg->data.c_str());

    /* write "Hi" to the FIFO */
    fd = open(myfifo, O_WRONLY);
    write(fd, &kingkong, sizeof(kingkong));
    close(fd);

    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  /* create the FIFO (named pipe) */
  if (loopChanger == 0) {mkfifo(myfifo, 0666); loopChanger = 1;}

  ros::NodeHandle n;

  signal(SIGINT, handler);

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}