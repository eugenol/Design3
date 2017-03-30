#include "crc.h"

static uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData) /*from avrlibc*/
{
	uint8_t   i;
	uint8_t   data;

	data = inCrc ^ inData;

	for ( i = 0; i < 8; i++ )
	{
		if (( data & 0x80 ) != 0 )
		{
			data <<= 1;
			data ^= 0x07;
		}
		else
		{
			data <<= 1;
		}
	}
	return data;
}

uint8_t CRC_Calculate(packet *p) /*should be 0 if crc id correct*/
{
	uint8_t crc = 0x00;
	uint8_t *ptr = (uint8_t*)p;
	
	for(uint8_t i= 1; i < 7; i++)
	{
		crc = _crc8_ccitt_update(crc, *(ptr+i));
	}
	return crc;
}
