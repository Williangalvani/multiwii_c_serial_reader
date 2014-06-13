/*
 * communication.cpp
 *
 *  Created on: 14/05/2014
 *      Author: fernando
 */

#include "communication.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */

//communication::communication() {
//	// TODO Auto-generated constructor stub
//
//}
//
//communication::~communication() {
//	// TODO Auto-generated destructor stub
//}

int communication::serialport_init(char *serialport, int baud) {
	struct termios toptions;
	int fd;
	fd = open(serialport, O_RDWR | O_NOCTTY);
	if (fd == -1) {
		perror("init_serialport: Unable to open port ");
		return -1;
	}

	if (tcgetattr(fd, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes");
		return -1;
	}

	speed_t brate = baud; // let you override switch below if needed
	switch (baud) {
	case 4800:
		brate = B4800;
		break;
	case 9600:
		brate = B9600;
		break;
	case 19200:
		brate = B19200;
		break;
	case 38400:
		brate = B38400;
		break;
	case 57600:
		brate = B57600;
		break;
	case 115200:
		brate = B115200;
		break;
	case 460800:
		brate = B460800;
		break;
	}

	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;

	// no flow control
	toptions.c_cflag &= ~CRTSCTS;
	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN] = 1;
	toptions.c_cc[VTIME] = 10;
	if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
		return -1;
	}

	return fd;
}

int communication::serialport_writebyte(int fd, uint8_t b) {
//	bool debugmode = true;
//    if(debugmode == true) printf("%d\n", b);
	int n = write(fd, &b, 1);
	if (n != 1)
		return -1;
	return 0;
}

int communication::serialport_write(int fd, const char* str) {
	int len = strlen(str);
	int n = write(fd, str, len);
	if (n != len)
		return -1;
	return n;
}

int communication::serialport_read_until(int fd, char buf[10][20], char until) {
	char b[1];
	int i = 0, j = 0;
	do {
		do {
			int n = read(fd, b, 1);  // read a char at a time
			if (n == -1)
				return -1;    // couldn't read
			if (n == 0) {
				usleep(10 * 1000); // wait 10 msec try again
				continue;
			}
			buf[j][i] = b[0];
			i++;
//			printf("%c\n", b[0]);
			if(b[0] == until)
				break;
		} while (b[0] != ',');
//		printf("Fim %d", j);
		buf[j][i] = 0;  // null terminate the string
		i = 0;
		j++;
	} while (b[0] != until);
	return i;
}

char communication::serialport_read(int fd) {
	char b[1];
	int n = read(fd, b, 1);  // read a char at a time
//	if (n == -1){
//		printf("erro ao ler serial\n");
//		return -1;    // couldn't read
//	}
	return b[0];
}

int communication::serialport_available(int fd) {
        
int nbytes = 0;
ioctl(fd, FIONREAD, &nbytes);
//if (nbytes)
//{
//printf("[%d]",nbytes);
//}
return nbytes;

}


