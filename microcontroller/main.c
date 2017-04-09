#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/crc16.h>
 
#define F_CPU 16000000UL
#define BAUD 9600

#include <util/setbaud.h>
#include <util/delay.h>

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
}packet;

enum{NACK = 0x00, ACK = 0xFF};
uint8_t volatile response = NACK;

/*** prototypes ***/
int32_t Sensor_ReadHall(void);
float Sensor_ReadTemp(void);

void ADC_init(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz

	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	//ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	// No MUX values needed to be changed to use ADC0

	//ADCSRA |= (1 << ADFR);  // Set ADC to Free-Running Mode
	//ADCSRA |= (1 << ADATE);		//NB need this to enable trigger mode
	ADCSRA |= (1 << ADEN);  // Enable ADC
	//ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	
	//ADCSRB - free running mode
	_delay_ms(1);
}

uint16_t ADC_read(uint8_t ADCchannel)
{
	ADMUX =(ADMUX & 0xF0) | (ADCchannel & 0x0F); //select ADC channel with safety mask
	ADCSRA |= (1<<ADSC);	//single conversion mode
	while(ADCSRA & (1<<ADSC));	//wait for conversion to complete.
	return ADC;
}


void USART_init(void)
{
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A |= (1 << U2X0);
#else
	UCSR0A &= ~(1 << U2X0);
#endif

	UCSR0B = (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void USART_transmit(uint8_t ch)
{
	while (!(UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer */
	UDR0 = ch;						/* Put data into buffer, sends the data */
}	

void USART_sendPacket(packet *p)
{
	char *ptr = (char*)p;
	response = NACK;
	
	do
	{
		for(uint8_t i = 0; i < PACKET_SIZE;i++)
		{
			USART_transmit(*(ptr+i));
		}
		_delay_ms(500);
	}while(response!=ACK);
}

ISR(USART_RX_vect)
{
	response = UDR0;
}

void CRC_calculate(packet *p)
{
	uint8_t crc = 0x00;
	uint8_t *ptr = (uint8_t*)p;
	for(uint8_t i= 1; i < PACKET_SIZE-2; i++)
	{
		crc = _crc8_ccitt_update(crc, *(ptr+i));
	}
	p->crc = crc;
}

int main (void)
{
	USART_init();
	ADC_init();
	sei(); //enable interrupts;

	packet testpacket;
	testpacket.start_byte =0x30;
	testpacket.sensor_id = 0x01;
	testpacket.message_id = 0;
	testpacket.data_type =0;
	testpacket.data_float = 1.0100118;
	testpacket.crc = 0xFF;
	testpacket.stop_byte = 0x20;
	
	//CRC_calculate(&testpacket);

	//USART_sendPacket(&testpacket);
	
	float ADCval;
	
	while(1)
	{
		testpacket.message_id++;
		testpacket.sensor_id = 0x01;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadHall();
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		testpacket.message_id++;
		testpacket.sensor_id = 0x02;
		testpacket.data_type = 0;
		testpacket.data_float = Sensor_ReadTemp();
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
	}
}

float Sensor_ReadTemp(void) {
	float temp = ADC_read(1);
	
	return (temp*5/((float)1024)*100);
}

int32_t Sensor_ReadHall(void) {
	return ADC_read(0);
}


/*
		ADCval = (ADCH*(5/(float)256))*100;
		testpacket.message_id++;
		testpacket.data_float = ADCval;
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(10000);
*/
