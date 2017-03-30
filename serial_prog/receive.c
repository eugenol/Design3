/**************************************************
file: receive.c
receive data packet over serial from sensor interface unit
exit the program by pressing Ctrl-C
compile with the command:
gcc receive.c rs232.c crc.c -Wall -Wextra -o2 -o receive
pass flag -DTESTING to enable output
**************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include "rs232.h" 		/*serial library*/
#include "receive.h"	/*definitions -  struct*/
#include "crc.h"		/*crc functiond*/

int main()
{
	int n,
		cport_nr=24,        /* /dev/ttyACM0 */
		bdrate=9600;       /* 9600 baud */

	unsigned char buf[4096];

	char mode[]={'8','N','1',0};

	packet p;
	
	if(RS232_OpenComport(cport_nr, bdrate, mode))
	{
		printf("Can not open comport\n");

		return(0);
	}
	
	while(1)
	{
		n = RS232_PollComport(cport_nr, buf, 4095);

		if(n == PACKET_SIZE)
		{
			memcpy(&p,buf,PACKET_SIZE); //copy buffer to packet structure
			
			printf("Start byte: %x\n", p.start_byte);
			printf("Message Id: %d\n", p.message_id);
			printf("Sensor Id: %d\n", p.sensor_id);
			printf("Data Type: %d\n", p.data_type);
			if(p.data_type==0)
			{
				printf("Reading: %f\n", p.data_float);
			}
			else
			{
				printf("Reading: %d\n", p.data_int);
			}
			
			printf("CRC: %x\n", p.crc);
			printf("Stop byte: %x\n", p.stop_byte);
	 
			// check start & stop bytes, check CRC
			if (buf[0]==0x30 && buf[PACKET_SIZE-1] ==0x20 && CRC_Calculate(&p)==0)
			{
				RS232_SendByte(cport_nr, ACK); //send ack
				printf("CRC match, ACK sent\n");	
			}
			else
			{
				RS232_SendByte(cport_nr, NACK); //send nack
				printf("CRC mismatch, NACK sent\n");		
			}
			printf("\n");
		}
		usleep(100000);  /* sleep for 100 milliSeconds */
	}

  return(0);
}
