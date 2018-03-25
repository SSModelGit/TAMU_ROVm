// https://stackoverflow.com/questions/2784500/how-to-send-a-simple-string-between-two-programs-using-pipes - where the original code was pulled from
// Contains (now altered) code that transmits an integer over a pipe
// This contains the reader part of the pipe - the client side

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAX_BUF 1024

int main()
{
    int fd;
    char * myfifo = "/tmp/myfifo";
    int buf;
    int nbuf;

    /* open, read, and display the message from the FIFO */
    fd = open(myfifo, O_RDONLY);
    read(fd, &buf, sizeof(buf));
    nbuf = nifty(buf);
    close(fd);
    printf("Nifty yodel: %d\n", nbuf);
    return 0;
}

int nifty(int i) {
    return i*2;
}