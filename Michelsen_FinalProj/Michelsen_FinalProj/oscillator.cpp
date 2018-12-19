/*
 * CPPFile1.cpp
 *
 * This code was based on work by Brent Edstrom in his book "Arduino for Musicians"
 * Citation:
 * Edstrom, B. (2016). Audio Output and Sound Synthesis. In Arduino for Musicians: A Complete Guide to Arduino and Teensy Microcontrollers. New York, NY: Oxford University Press. 
 */ 

#include "oscillator.h"

/* Base Oscillator Class Definitions */
// Default constructor
OscillatorBase::OscillatorBase()
{
	// Initialize the oscillator
	init();
}

// Initializer function
void OscillatorBase::init()
{
	m_sampleRate = 16384;
	m_frequency = 440.0;
	m_maxAmp = 255;
	m_max32 = 4294967295UL;
	m_increment = 1;
	m_accumulator = 0;
}

// Set sample rate
void OscillatorBase::setSampleRate(uint16_t rate)
{
	m_sampleRate = rate;
}

// Set frequency
void OscillatorBase::setFreq(float freq)
{
	m_frequency = freq;
	if (m_frequency == 0)
	m_frequency = 1; // Avoid divide by 0
	
	m_increment = calcIncrement();
	if (m_increment == 0)
	m_increment = 1;
}
/* End Base Oscillator Class Definitions */

/* Ramp Oscillator Class Definitions */

/* End Ramp Oscillator Class Definitiions */