#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdint.h>

/*struct must be packed to align with data coming from
 * the microcontroller */
 
struct datapacket{
	uint8_t start;
	uint8_t id;
	float data; /*4 bytes*/
	uint8_t crc;
	uint8_t stop;
} __attribute__((packed));

typedef struct datapacket packet;

enum{NACK = 0x00, ACK =0xFF};

#endif /*RECEIVE_H*/
