#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
 
#define F_CPU 8000000UL
#define BAUD 9600

#include <util/setbaud.h>
#include <util/delay.h>

uint8_t level = 0;
uint8_t last_level = 0;
uint8_t sensors[5] = {0};

/*** prototypes ***/
void SPI_init(void);
void measureLevel(void);

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

void ADC_init(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz

	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	// No MUX values needed to be changed to use ADC0

	//ADCSRA |= (1 << ADFR);  // Set ADC to Free-Running Mode
	//ADCSRA |= (1 << ADATE);		//NB need this to enable trigger mode
	ADCSRA |= (1 << ADEN);  // Enable ADC
	//ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	
	//ADCSRB - free running mode
	_delay_ms(1);
}

uint8_t ADC_read(uint8_t ADCchannel)
{
	ADMUX =(ADMUX & 0xF0) | (ADCchannel & 0x0F); //select ADC channel with safety mask
	ADCSRA |= (1<<ADSC);	//single conversion mode
	while(ADCSRA & (1<<ADSC));	//wait for conversion to complete.
	return ADCH;
}


// SPI Transmission/reception complete ISR
ISR(SPI_STC_vect)
{
	uint8_t sensor = SPDR;
	SPDR = level; //sensors[sensor];
}

int main (void)
{
	USART_init();
	SPI_init();
	ADC_init();
	_delay_ms(1000);
	sei(); //enable interrupts;

	level = 0;
	
	//USART_transmit(255);
	
	while(1)
	{
		measureLevel();
	}
}

// SPI Slave Init
void SPI_init(void) {
	//DDRB=(1<<4); //set MISO as ouput
	//SPCR=(1<<SPE);//|(1<<SPIE); // Enable SPI, set as slave, enable interrupts.
	DDRB &= ~((1<<2)|(1<<3)|(1<<5));   // SCK, MOSI and SS as inputs
    DDRB |= (1<<4);                    // MISO as output

    SPCR &= ~(1<<MSTR);                // Set as slave 
    //SPCR |= (1<<SPR0)|(1<<SPR1);       // divide clock by 128
    SPCR |= (1<<SPIE);                 // Enable SPI Interrupt
    SPCR |= (1<<SPE);                  // Enable SPI
}

void measureLevel(void)
{
	last_level = level;
	
	for (uint8_t i = 0; i < 5; i++)
	{
		sensors[i] = ADC_read(i+1);
		sensors[i] = ((sensors[i]>=130)?(sensors[i]-130):0);
	}
	
	uint8_t max_pos = 0;
	uint8_t max_val = sensors[0];
	
	for (uint8_t i = 1; i < 5; i++)	
	{
		if (sensors[i] > max_val)
		{
			max_pos = i;
			max_val = sensors[i];
		}	
	}
	
	if (max_pos == 0 && max_val ==0)
		max_pos = 5;
	
	switch(max_pos)
	{
		case 0:
			level = 1;
			break;
		case 1:
			level = 25;
			break;
		case 2:
			level = 50;
			break;
		case 3:
			level = 75;
			break;
		case 4:
			level = 100;
			break;
		case 5:
			level = last_level;
	}
	
	USART_transmit(level);
}
