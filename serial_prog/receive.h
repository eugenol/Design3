#ifndef RECEIVE_H
#define RECEIVE_H

#include <stdint.h>

/*struct must be packed to align with data coming from
 * the microcontroller */
 
#define PACKET_SIZE 10
 
typedef struct datapacket{
	uint8_t start_byte;
	uint8_t message_id;
	uint8_t sensor_id;
	uint8_t data_type;
	union {
		float data_float; /*4 bytes*/
		int32_t data_int;
	};
	uint8_t crc;
	uint8_t stop_byte;
} __attribute__((packed)) packet;

enum{NACK = 0x00, ACK =0xFF};

#endif /*RECEIVE_H*/
