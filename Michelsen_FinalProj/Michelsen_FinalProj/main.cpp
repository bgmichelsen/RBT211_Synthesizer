/*
 * Michelsen_FinalProj.cpp
 *
 * Created: 12/2/2018 8:09:41 PM
 * Author : Brandon Michelsen
 */ 

#define F_CPU	16000000UL // 16 MHz clock

#define SAMPLE	16384 // Sample rate of the oscillator

/* Defines for 74HC165 pins */
#define SHLD	PORTB0 // Shift/load control pin
#define CLK		PORTB1 // Serial clock
#define INH		PORTB2 // Clock inhibit pin
#define QH		PINB3 // Serial input pin
#define CHIPS	1 // Number of connected chips
#define WIDTH	CHIPS*8 // Data width of the shift registers

/* Defines for the musical scales */
#define NUMSCALES	4
#define NUMNOTES	8

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <m328/oscillator/oscillator.h>
#include "Tones.h"

// Function prototypes
void initTimer1(); // Function for initializing timer 1
uint8_t shiftIn(); // Function for reading buttons from 74HC165
int _map(int input, int x1, int x2, int y1, int y2); // Function for mapping one numeric range to another
int adc_read_10(int channel); // Analog read function for the ADC
void setOscFreq(int oscSelect, float freq); // Function to set a given oscillator's frequency
uint8_t oscTick(int oscSelect1, int oscSelect2); // Function to mix the oscillators

// Setup oscillators
//DescedningRampOscillator osc1;
//SineOscillator osc2;

DescedningRampOscillator dRamp1;
AscendingRampOscillator aRamp1;
TriangleOscillator triangle1;
PWMOscillator sqWave1;
SineOscillator sine1;

DescedningRampOscillator dRamp2;
AscendingRampOscillator aRamp2;
TriangleOscillator triangle2;
PWMOscillator sqWave2;
SineOscillator sine2;
SineOscillator lfo;

// Define the musical scales to be used (in EEPROM)
const float scales[NUMSCALES][NUMNOTES] PROGMEM = {
	{ NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5 }, // C Major
	{ NOTE_A3, NOTE_B3, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4 }, // A Minor
	{ NOTE_D4, NOTE_E4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_CS5, NOTE_D5 }, // D Minor
	{ NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_FS5, NOTE_G5 } // G Major
};

// Global variables
volatile int osc1 = 0; // Variable for tracking the first oscillator selection
volatile int osc2 = 0; // Variable for tracking the second oscillator selection
volatile bool audioOn = false; // Variable for tracking if the audio is being played or not

int main(void)
{
	// Setup inputs and outputs
	DDRD |= 0xFF; // Set all of PORTD as outputs
	DDRB |= ((1<<DDB2) | (1<<DDB1) | (1<<DDB0)); // Setup outputs for shift register
	DDRB &= ~(1<<DDB3); // Setup inputs from shift register
	
	// Setup timer and global interrupts
	cli();
	initTimer1();
	sei();
	
	/* Set the sample rate of the oscillators */
	lfo.setSampleRate(SAMPLE);
	//osc1.setSampleRate(SAMPLE);
	//osc2.setSampleRate(SAMPLE);
	dRamp1.setSampleRate(SAMPLE);
	aRamp1.setSampleRate(SAMPLE);
	triangle1.setSampleRate(SAMPLE);
	sqWave1.setSampleRate(SAMPLE);
	sine1.setSampleRate(SAMPLE);
	
	dRamp1.setSampleRate(SAMPLE);
	aRamp1.setSampleRate(SAMPLE);
	triangle1.setSampleRate(SAMPLE);
	sqWave1.setSampleRate(SAMPLE);
	sine1.setSampleRate(SAMPLE);
	
	// Set the duty cycle of the two square wave oscillators
	sqWave1.setDutyCycle(0.5);
	sqWave2.setDutyCycle(0.5);
	
	lfo.setFreq(7); // Set the frequency of the LFO (used for vibrato effect)
	
	// Declare local variables
	float oscNote = 0; // Variable for tracking the note to be played
	int keySelector = 0; // Variable for selecting the key of the music
	uint8_t buttonByte = 0; // Byte for reading in the keyboard presses
	int lfoTick = 0; // Variable for tracking the lfo ticks
	
    // Run infinitely
    while (1) 
    {
		// Read the key selector
		keySelector = _map(adc_read_10(0), 0, 1023, 0, 4);
		_delay_ms(10);
		keySelector = _map(adc_read_10(0), 0, 1023, 0, 4);
		
		// Read the oscillator selectors
		osc1 = _map(adc_read_10(1), 0, 1023, 0, 5);
		_delay_us(10);
		osc1 = _map(adc_read_10(1), 0, 1023, 0, 5);
		_delay_ms(10);
		osc2 = _map(adc_read_10(2), 0, 1023, 0, 5);
		_delay_ms(10);
		osc2 = _map(adc_read_10(2), 0, 1023, 0, 5);
				
		// Read the buttons
		buttonByte = shiftIn();
		
		// If a button is pressed, play a note
		if (buttonByte != 0)
		{
			// Select the note based on the button pressed
			if (buttonByte & 0b00000001)
				oscNote = pgm_read_float_near(&(scales[keySelector][0]));
			if (buttonByte & 0b00000010)
				oscNote = pgm_read_float_near(&(scales[keySelector][1]));
			if (buttonByte & 0b00000100)
				oscNote = pgm_read_float_near(&(scales[keySelector][2]));
			if (buttonByte & 0b00001000)
				oscNote = pgm_read_float_near(&(scales[keySelector][3]));
			if (buttonByte & 0b00010000)
				oscNote = pgm_read_float_near(&(scales[keySelector][4]));
			if (buttonByte & 0b00100000)
				oscNote = pgm_read_float_near(&(scales[keySelector][5]));
			if (buttonByte & 0b01000000)
				oscNote = pgm_read_float_near(&(scales[keySelector][6]));
			if (buttonByte & 0b10000000)
				oscNote = pgm_read_float_near(&(scales[keySelector][7]));
			
			// Modulate the pitch by the LFO (to get a vibrato effect)
			lfoTick = (lfo.tick() >> 6);
			
			// Set the frequencies of the two oscillators modulated by the LFO
			//osc1.setFreq(oscNote + lfoTick);
			//osc2.setFreq(oscNote*2 + lfoTick);
			setOscFreq(osc1, (oscNote + lfoTick));
			setOscFreq(osc2, (oscNote*2 + lfoTick));
			
			// Set the audio on to true
			audioOn = true;
		}
		else
		{
			// Otherwise, don't play a note
			//osc1.setFreq(0);
			//osc2.setFreq(0);
			setOscFreq(osc1, 0);
			setOscFreq(osc2, 0);
			audioOn = false;
		}
    }
}

// Function to set up Timer 1
void initTimer1()
{
	// Reset all timer values
	TCCR1A = 0;
	TCCR1B = 0;
	
	// Set the OCR value
	// (F_CPU / sampleRate)
	OCR1A = 975;
	
	// Set up timer for CTC mode, no prescale
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<CS10);
	
	// Enable timer interrupt
	TIMSK1 |= (1<<OCIE1A); 
}

// Function for reading button presses from the 74HC165 shift register
uint8_t shiftIn()
{
	// Declare local variables
	bool bitVal = 0; // Variable for tracking the bit value in the serial series
	uint8_t byteVal = 0; // Variable for the final byte received from the register
	
	// Load values from parallel lines
	PORTB |= (1<<INH); 
	PORTB &= ~(1<<SHLD);
	_delay_us(5);
	PORTB |= (1<<SHLD);
	PORTB &= ~(1<<INH);
	
	// Read data from the serial line
	for (uint8_t i = 0; i < WIDTH; ++i)
	{
		bitVal = (PINB & (1<<QH));
		byteVal |= (bitVal << (((WIDTH - 1) - i))); // Read data from MSB
		
		// Pulse the clock
		PORTB |= (1<<CLK);
		_delay_us(5);
		PORTB &= ~(1<<CLK);
	}
	
	return byteVal;
}

// Function to map one numeric range to another
int _map(int input, int x1, int x2, int y1, int y2)
{
	return ((input - x1)*((float)(y2-y1)/(x2-x1))+y1);
}

int adc_read_10(int channel)
{
	// Set digital inputs to be off on PORTC
	DIDR0 = 0x3F;
	
	// Read the proper channel
	if (channel == 0) ADMUX = 0x00; // Channel 0
	if (channel == 1) ADMUX = 0x01; // Channel 1
	if (channel == 2) ADMUX = 0x02; // Channel 2
	if (channel == 3) ADMUX = 0x03; // Channel 3
	if (channel == 4) ADMUX = 0x04; // Channel 4
	if (channel == 5) ADMUX = 0x05; // Channel 5
	
	// Read the analog value
	ADCSRA = ((1<<ADEN) | (1<<ADATE));
	ADCSRA |= (1<<ADSC);
	while (ADSC == 1);
	return (ADCL + (256*ADCH));
}

// Function to set a given oscillators frequency
void setOscFreq(int oscSelect, float freq)
{
	if (oscSelect == osc1)
	{
		if (oscSelect == 0)
			dRamp1.setFreq(freq);
		if (oscSelect == 1)
			aRamp1.setFreq(freq);
		if (oscSelect == 2)
			triangle1.setFreq(freq);
		if (oscSelect == 3)
			sqWave1.setFreq(freq);
		if (oscSelect == 4)
			sine1.setFreq(freq);
	}
	else if (oscSelect == osc2)
	{
		if (oscSelect == 0)
			dRamp2.setFreq(freq);
		if (oscSelect == 1)
			aRamp2.setFreq(freq);
		if (oscSelect == 2)
			triangle2.setFreq(freq);
		if (oscSelect == 3)
			sqWave2.setFreq(freq);
		if (oscSelect == 4)
			sine2.setFreq(freq);
	}
	else
	{
		return;
	}
}

// Function to mix the oscillators
uint8_t oscTick(int oscSelect1, int oscSelect2)
{
	// Declare local variables
	uint8_t val = 0; // Tracker for return value
	
	// Select the proper oscillator
	if (oscSelect1 == 0)
	{
		if (oscSelect2 == 0)
			val = (dRamp1.tick() + (dRamp2.tick() >> 3));
		if (oscSelect2 == 1)
			val = (dRamp1.tick() + (aRamp2.tick() >> 3));
		if (oscSelect2 == 2)
			val = (dRamp1.tick() + (triangle2.tick() >> 3));
		if (oscSelect2 == 3)
			val = (dRamp1.tick() + (sqWave2.tick() >> 3));
		if (oscSelect2 == 4)
			val = (dRamp1.tick() + (sine2.tick() >> 3));
	}
	else if (oscSelect1 == 1)
	{
		if (oscSelect2 == 0)
			val = (aRamp1.tick() + (dRamp2.tick() >> 3));
		if (oscSelect2 == 1)
			val = (aRamp1.tick() + (aRamp2.tick() >> 3));
		if (oscSelect2 == 2)
			val = (aRamp1.tick() + (triangle2.tick() >> 3));
		if (oscSelect2 == 3)
			val = (aRamp1.tick() + (sqWave2.tick() >> 3));
		if (oscSelect2 == 4)
			val = (aRamp1.tick() + (sine2.tick() >> 3));
	}
	else if (oscSelect1 == 2)
	{
		if (oscSelect2 == 0)
			val = (triangle1.tick() + (dRamp2.tick() >> 3));
		if (oscSelect2 == 1)
			val = (triangle1.tick() + (aRamp2.tick() >> 3));
		if (oscSelect2 == 2)
			val = (triangle1.tick() + (triangle2.tick() >> 3));
		if (oscSelect2 == 3)
			val = (triangle1.tick() + (sqWave2.tick() >> 3));
		if (oscSelect2 == 4)
			val = (triangle1.tick() + (sine2.tick() >> 3));
	}
	else if (oscSelect1 == 3)
	{
		if (oscSelect2 == 0)
			val = (sqWave1.tick() + (dRamp2.tick() >> 3));
		if (oscSelect2 == 1)
			val = (sqWave1.tick() + (aRamp2.tick() >> 3));
		if (oscSelect2 == 2)
			val = (sqWave1.tick() + (triangle2.tick() >> 3));
		if (oscSelect2 == 3)
			val = (sqWave1.tick() + (sqWave2.tick() >> 3));
		if (oscSelect2 == 4)
			val = (sqWave1.tick() + (sine2.tick() >> 3));
	}
	else if (oscSelect1 == 4)
	{
		if (oscSelect2 == 0)
			val = (sine1.tick() + (dRamp2.tick() >> 3));
		if (oscSelect2 == 1)
			val = (sine1.tick() + (aRamp2.tick() >> 3));
		if (oscSelect2 == 2)
			val = (sine1.tick() + (triangle2.tick() >> 3));
		if (oscSelect2 == 3)
			val = (sine1.tick() + (sqWave2.tick() >> 3));
		if (oscSelect2 == 4)
			val = (sine1.tick() + (sine2.tick() >> 3));
	}
	else
	{
		val = 0;
	}
	
	// Return the value
	return val;
}
// Timer 1 Interrupt (for sampling the waveform)
ISR(TIMER1_COMPA_vect)
{
	// Output DAC audio (additive synthesis)
	//PORTD = (osc1.tick() + (osc2.tick() >> 3));
	if (audioOn == true)
		PORTD = oscTick(osc1, osc2);
	else
		PORTD = 0;
}