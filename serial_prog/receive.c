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
/*NEW*/
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
/*END NEW*/


int main()
{
	/*NEW*/
	int clientSocket;
	unsigned char buffer[4096];
	struct sockaddr_in serverAddr;
	socklen_t addr_size;


	//configure the socket settings for the server it will be connecting to
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(4000);
	serverAddr.sin_addr.s_addr = inet_addr("10.0.0.5");
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);
	/*END NEW*/

	int n,
		cport_nr=22,        /* /dev/ttyACM0 */
		bdrate=9600;       /* 9600 baud */

	uint8_t prev_message_id = 255;
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
				//over here, check if message_id is the same as the previus message id
				//if it is, then dont transmit the packet to the server
				//the microcontroller may not have received the ACK signal.
				printf("CRC match, ACK sent\n");	

				if(prev_message_id != p.message_id)
				{
					/*NEW*/
					memcpy(buffer, &p,PACKET_SIZE); //copy packet stru$
					//create a socket
					clientSocket = socket(PF_INET, SOCK_STREAM, 0);

					addr_size = sizeof serverAddr;
					connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size); //Connect to the server
					send(clientSocket,buffer,50,0); //send the packet
					/*END NEW*/

					printf("Send to socket\n");
					prev_message_id = p.message_id;
				}

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
