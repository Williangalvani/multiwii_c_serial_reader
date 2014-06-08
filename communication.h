/*
 * communication.h
 *
 *  Created on: 14/05/2014
 *      Author: fernando
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <stdint.h>   /* Standard types */

class communication {
	int baudrate;
	char type;
	char destination[2];
	char *message;
	char *localUsb;

//	int fd; //= 0;
//	char serialport[256];
//	char buf[20], dat[20], use[1];
//	int rc,n;
//	bool debugmode =true;

public:
//	communication();
//	virtual ~communication();
	int serialport_init(char *serialport, int baud);
	int serialport_writebyte(int fd, uint8_t b);
	int serialport_write(int fd, const char* str);
	int serialport_read_until(int fd, char buf[10][20], char until);
	char serialport_read(int fd);
        int serialport_available(int fd, char* c);
};

#endif /* COMMUNICATION_H_ */

