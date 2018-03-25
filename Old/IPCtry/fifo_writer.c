// https://stackoverflow.com/questions/2784500/how-to-send-a-simple-string-between-two-programs-using-pipes - where the original code was pulled from
// Contains (now altered) code that transmits an integer over a pipe
// This contains the writer part of the pipe - the server side

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int main()
{
    int fd;
    char * myfifo = "/tmp/myfifo";
    int kingkong = 1;

    /* create the FIFO (named pipe) */
    mkfifo(myfifo, 0666);

    /* write "Hi" to the FIFO */
    fd = open(myfifo, O_WRONLY);
    write(fd, &kingkong, sizeof(kingkong));
    close(fd);

    /* remove the FIFO */
    unlink(myfifo);

    return 0;
}