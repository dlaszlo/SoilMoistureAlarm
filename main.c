#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

ISR(PCINT0_vect)
{
}

ISR(WDT_vect)
{
}

/*
 * 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
 * 6=1s,7=2s, 8=4s, 9=8s
 */
void setup_watchdog(uint8_t ii)
{
	uint8_t bb;
	if (ii > 9) ii = 9;
	bb = ii & 7;
	if (ii > 7) bb |= (1 << 5);
	bb |= (1 << WDCE);
	cli();
	MCUSR &= ~(1 << WDRF);
	// Start timed sequence
	WDTCR |= (1 << WDCE) | (1 << WDE);
	// Set new watchdog timeout value
	WDTCR = bb;
	WDTCR |= _BV(WDIE);
}

/*
 * Set system into the sleep state
 * System wakes up when watchdog is timed out
 */
void system_sleep()
{
	cbi(ADCSRA, ADEN);                   // Switch Analog to Digitalconverter OFF
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Sleep mode is set here
	sei();
	sleep_mode();                        // System sleeps here
	sbi(ADCSRA, ADEN);                   // Switch Analog to Digitalconverter ON
}

uint16_t readAnalog()
{
	// Set prescaler to div64 --> 125kHz ADC clock
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1);
	// Start a conversion
	ADCSRA |= (1<<ADSC);
	// Wait for end of conversion
	while (ADCSRA & (1<<ADSC))
	{
		asm volatile ("nop");
	}
	uint8_t l = ADCL;// Datasheet says read low first
	uint8_t h = ADCH;
	return (h << 8) | l;
}

void setup()
{
	ADMUX = (1<<MUX1) | (1<<MUX0); // MUX set to ADC3. VCC as ref. Right justified
	ADCSRA = (1<<ADEN);

	DDRB = 0b00010111;

	sbi(GIMSK,PCIE); // Turn on Pin Change interrupt
	sbi(PCMSK,PCINT0);

	setup_watchdog(9);
}

int main(void)
{
	setup();

	for (;;)
	{
		PORTB = 0b00000011;
		uint16_t v = readAnalog();

		PORTB = 0b00000101;
		_delay_ms(1);

		if (v < 512)
		{
			PORTB = 0b00010001;
			_delay_ms(200);
		}
		PORTB = 0b00000001;
		if (PINB & (1<<PB0)) {
			system_sleep();
		}
	}
}
