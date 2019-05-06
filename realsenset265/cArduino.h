#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#define _POSIX_SOURCE 1

class serial{
public:


void init(std::string dev ){
    fd = open(dev.c_str(), O_RDWR | O_NOCTTY );
    if (fd <0) {perror(dev.c_str()); exit(-1); } 
    // Improvement No. 1 I save old setting and clean the new one 
    tcgetattr(fd,&oldtio);
    bzero(&newtio, sizeof(newtio));

    // Here I set all the flags to vars at once
    newtio.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;
    //here I set some new flags..
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character  arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */
    // and I finally save the settings 
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
}

~serial(){
    close(fd);
}

    std::string sread(){
        res = read(fd,buf,255);
        buf[res]=0;
        return buf;
    }
    void swrite(const char* input){
        write(fd,input,sizeof(input));
    }
private:
    int fd,c,res;
    struct termios oldtio,newtio;
    char buf[255];
};
