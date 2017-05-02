#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/crc16.h>
#include <math.h>
 
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
		int32_t data_int; /*4 bytes*/
	};
	uint8_t crc;
	uint8_t stop_byte;
}packet;

uint8_t ack_check[256] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 
							4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4,
							4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3,
							4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 
							4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 
							4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 
							4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 
							4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 
							4, 5, 5, 6, 5, 6, 6, 7, 0, 1, 1, 2, 1, 2, 2, 
							3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 
							3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 
							3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 
							3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 
							3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 
							3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 
							3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
							3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7};

enum{NACK = 0x00, ACK = 0xFF};
uint8_t volatile response = NACK;

/*** prototypes ***/
void SPI_init(void);
uint8_t SPI_tranceive(uint8_t data);

// Sensor Prototypes
int32_t Sensor_ReadBundTap(void);
int32_t Sensor_ReadFlame(void);
float Sensor_ReadTemp(void);
int32_t Sensor_ReadBundLevel(void);
int32_t Sensor_ReadBundLevel_Test(uint8_t s);

void IO_init(void) {
	DDRD = 0x70; //set pins 4,5,6 on port D as output;
}

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
	uint8_t temp = UDR0;
	response = ack_check[temp] > 3? ACK : NACK;
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
	IO_init();
	SPI_init();
	sei(); //enable interrupts;

	packet testpacket;
	testpacket.start_byte =0x30;
	testpacket.sensor_id = 0x01;
	testpacket.message_id = 0;
	testpacket.data_type =0;
	testpacket.data_float = 0;
	testpacket.crc = 0;
	testpacket.stop_byte = 0x20;
	
	_delay_ms(2000); 
	
	while(1)
	{
		// Temperature Sensor
		testpacket.message_id++;
		testpacket.sensor_id = 0x00;
		testpacket.data_type = 0;
		testpacket.data_float = Sensor_ReadTemp();
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		// Flame Sensor
		testpacket.message_id++;
		testpacket.sensor_id = 0x01;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadFlame();
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		// Tap sensor
		testpacket.message_id++;
		testpacket.sensor_id = 0x02;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundTap();
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		// Level Sensor
		testpacket.message_id++;
		testpacket.sensor_id = 0x03;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundLevel();
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(1000);
		/*
		testpacket.message_id++;
		testpacket.sensor_id = 0x30;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundLevel_Test(0x00);
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		testpacket.message_id++;
		testpacket.sensor_id = 0x31;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundLevel_Test(0x01);
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		testpacket.message_id++;
		testpacket.sensor_id = 0x32;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundLevel_Test(0x02);
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		testpacket.message_id++;
		testpacket.sensor_id = 0x33;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundLevel_Test(0x03);
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);
		
		testpacket.message_id++;
		testpacket.sensor_id = 0x34;
		testpacket.data_type = 1;
		testpacket.data_int = Sensor_ReadBundLevel_Test(0x04);
		CRC_calculate(&testpacket);
		USART_sendPacket(&testpacket);
		_delay_ms(500);*/
	}
}

int32_t Sensor_ReadFlame(void) {
	uint16_t res = ADC_read(1);
	if (res > 400)
		return 0;
	else //if (res >= 500 )
		return 1;
}

float Sensor_ReadTemp(void) {
	/* R0 = 9740 ohms
	 * B = 3435
	 * T0 = 298.15 K
	 * r = R0*(ADC_MAX/ADC_VAL -1)
	 * T = B/ln(r/R1)
	 * R1 = R0*e^(-B/T0)
	 */
	 
	float temp = ADC_read(2);
	
	temp = 9740*(1023.0/temp -1);
	temp /= 0.0919;
	temp = log(temp);
	temp = 3435/temp;
	
	return (temp - 273.15);
}

int32_t Sensor_ReadBundTap(void) {
	uint16_t res = ADC_read(0);
	if (res >= 610)
		return 0;
	else
		return 1;
}

int32_t Sensor_ReadBundLevel(void) {
	PORTB &= ~(1<<2);		
	uint8_t ret = SPI_tranceive(0x00);
	if (ret==255)
		return -1;
	else
		return ret;
}

int32_t Sensor_ReadBundLevel_Test(uint8_t s) {
	PORTB &= ~(1<<2);		
	uint8_t ret = SPI_tranceive(s);
	return ret;
}

void SPI_init(void) {
	//DDRB=(1<<3)|(1<<5)|(1<<2); //set MOSI and SCK and SS as ouput.
	//SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0);//|(1<<SPIE); // Enable SPI, set as master. Prescaler = Fosc/16
	//PORTB &= ~(1<<PB2);
	
	DDRB |= (1<<2)|(1<<3)|(1<<5);    // SCK, MOSI and SS as outputs
    DDRB &= ~(1<<4);                 // MISO as input

    SPCR |= (1<<MSTR);               // Set as Master
    //SPCR |= (1<<SPR0)|(1<<SPR1);     // divided clock by 128
    SPCR |= (1<<SPR0);//|(1<<SPR1);
    //SPCR |= (1<<SPIE);               // Enable SPI Interrupt    
    SPCR |= (1<<SPE);                // Enable SPI	
}

uint8_t SPI_tranceive(uint8_t data) {
	SPDR = data; //load data into buffer
	while(!(SPSR & (1<<SPIF))); //wait for transmission to complete
	return(SPDR); //return received data
}
