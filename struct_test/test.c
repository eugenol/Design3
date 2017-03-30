#include <stdio.h>

int main(int argc, char **argv)
{
	struct datapacket{
		char start;
		char id;
		char data[4];
		char crc;
		char stop;
	}/*__attribute__((packed))*/;

	typedef struct datapacket packet;

	unsigned char data[8] = {0x30, 0x01, 0x11, 0x48, 0x81, 0x3f, 0xff, 0x20};

	packet *ptr = (packet*)data;
	float *fptr = (float*)ptr->data;

	printf("%d\n",ptr->start);
	printf("%f\n", *fptr);
	printf("%d\n", ptr->stop);

	//float f = 1.0;
	//char* fptr = (char*)&f;

	//printf("%x %x %x %x\n", fptr, fptr+1,fptr+2, fptr+3);

	printf("Struct size: %d\n", sizeof(packet));

	printf("Float size: %d\n", sizeof(float));

	return 0;
}
