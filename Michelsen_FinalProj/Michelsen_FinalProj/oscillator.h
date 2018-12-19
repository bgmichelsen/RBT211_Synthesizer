/*
 * oscillator.h
 *
  * This code was based on work by Brent Edstrom in his book "Arduino for Musicians"
  * Citation:
  * Edstrom, B. (2016). Audio Output and Sound Synthesis. In Arduino for Musicians: A Complete Guide to Arduino and Teensy Microcontrollers. New York, NY: Oxford University Press.
 */ 


#ifndef OSCILLATOR_H_
#define OSCILLATOR_H_

#include <avr/io.h>
#include "sinetable.h"

/* Base Oscillator Class */
class OscillatorBase
{
	protected:
	// Member variables
	uint16_t m_sampleRate; // Sample rate for the waveform
	float m_frequency; // Frequency of the oscillator
	uint16_t m_maxAmp; // Maximum amplitude (8-bit waveform = 255)
	uint32_t m_max32; // Maximum value of a 32-bit integer
	
	volatile uint32_t m_accumulator; // 32-bit accumulator
	volatile uint32_t m_increment; // 32-bit increment value
	
	public:
	// Default constructor
	OscillatorBase();
	
	// Getters and Setters
	virtual void setFreq(float freq);
	void setSampleRate(uint16_t rate);
	
	//	Helper methods
	void init();
	
	/* Inline Functions */
	// Calculate the increment from the frequency
	inline
	uint32_t calcIncrement()
	{
		return (uint32_t)((m_frequency*m_max32 + m_sampleRate/2)/m_sampleRate);
	}
	
	// Increment the accumulator
	inline
	virtual uint8_t tick()
	{
		m_accumulator += m_increment;
		return (m_accumulator >> 24); // Get the 8-bit real number
	}
};
/* End Base Oscillator Class */

/* Ascending Ramp Oscillator Class */
class AscendingRampOscillator: public OscillatorBase // Inherits from the Base Oscillator
{
	public:
	// Override base class tick() function
	inline
	virtual uint8_t tick()
	{
		// Return the accumulator value
		return OscillatorBase::tick();
	}
};
/* End Ascending Ramp Oscillator Class */

/* Descending Ramp Oscillator Class */
class DescedningRampOscillator: public OscillatorBase
{
	public:
	// Override base class tick() function
	virtual uint8_t tick()
	{
		// Tick the accumulator
		OscillatorBase::tick();
		
		// Return the wave starting from the highest value
		return ((m_max32 - m_accumulator) >> 24);
	}	
};
/* End Descending Ramp Class */

/* Square Wave Oscillator Class */
class PWMOscillator: public OscillatorBase
{
	protected:
	// Declare local variables
	uint32_t m_dutyCycle; // Duty cycle of the wave in relation to counter
	
	public:
	// Override default constructor
	PWMOscillator():OscillatorBase()
	{
		m_dutyCycle = m_max32 * 0.5; // Default duty cycle = 50%
	}	
	
	// Set the duty cycle
	void setDutyCycle(float duty)
	{
		m_dutyCycle = duty * m_max32;
	}
	
	// Override the base class tick() function
	inline
	virtual uint8_t tick()
	{
		// Tick the accumulator
		OscillatorBase::tick();
		
		// Get the pulse
		if (m_accumulator < m_dutyCycle)
			return (m_max32 >> 24);
		else
			return 0;
	}
};
/* End Square Wave Oscillator */

/* Triangle Oscillator Class */
class TriangleOscillator: public OscillatorBase
{
	protected:
	// Declare local variables
	uint32_t m_triangleAcc;
	
	public:
	// Override constructor to initialize triangle accumulator
	TriangleOscillator():OscillatorBase()
	{
		m_triangleAcc = 0;
	}
	
	// Override base class setFreq to calculate the the triangle wave values
	virtual void setFreq(uint16_t freq)
	{
		// Set base accumulator
		OscillatorBase::setFreq(freq);
		
		// Set up triangle accumulator (increments twice as fast as base accumulator)
		m_triangleAcc = 0;
	}
	
	// Override base tick function for triangle accumulator
	virtual uint8_t tick()
	{
		// Call base tick method
		OscillatorBase::tick();
		
		// Compare accumulator against half the full amplitude
		if (m_accumulator <= (m_max32 >> 1))
			// Multiply triangle accumulator by 2 while we are less than the full amplitude
			m_triangleAcc = (m_accumulator << 1);
		else
			// Otherwise, decrement the triangle accumulator
			m_triangleAcc = ((m_max32 - m_accumulator) << 1);
		return (m_triangleAcc >> 24);
	}
};
/* End Triangle Oscillator */

/* Noise generator */
class NoiseGen: public OscillatorBase
{
	protected:
	// Declare local variables
	uint16_t m_randomSample; // Used to sample the random data
	
	public:
	// Constructor
	NoiseGen():OscillatorBase()
	{
		m_randomSample = 1;
	}
	
	// Override base tick() function to return random values
	// use linear congruential method (Rnew = (A*Rold + B) mod M, mod is handled by ignoring the integer overflow
	virtual uint8_t tick()
	{
		m_randomSample = (77 * m_randomSample) + 55;
		return m_randomSample;
	}
};
/* End Noise Generator */

/* Sine Wave Oscillator Class */
class SineOscillator: public OscillatorBase
{
	public:
	// Override the base tick() function
	virtual uint8_t tick()
	{
		// Get the index of the current sample from the look up table
		uint8_t index = OscillatorBase::tick();
		
		// Get the byte representing the sample
		return (uint8_t)pgm_read_byte_near(sine+index);
	}	
};
/* End Sine Wave Oscillator */

#endif /* OSCILLATOR_H_ */