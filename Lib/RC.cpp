
#include "arduino.h"

#include "RC.h"

/**
 * default constructor
 */
RC::RC()
{
	;
}

void RC::printRCInCSV()
{
	;
}

void RC::Initialize()
{
	/**
	 * initialize each rc pin for PCINT PCINT activation.
	 * for more info about PCINT, look at
	 * <http://www.51hei.com/bbs/dpj-47867-1.html>
	 */
	for (uint8_t i = 0; i < PCINT_PIN_COUNT; i++) { // i think a for loop is ok for the init.
		PCINT_RX_PORT |= PCInt_RX_Pins[i];
		PCINT_RX_MASK |= PCInt_RX_Pins[i];
	}
	PCICR = PCIR_PORT_BIT;

	/*========temporarliy do not support aux pins========*/
}

void RC::getValue()
{
	;
}

uint16_t RC::readRCRaw()
{
	;
}

inline void pinCheckState(uint8_t pin_pos, uint8_t rc_value_pos)
{
	if (mask & PCInt_RX_Pins[pin_pos])
	{
		//if the pin is at low-level, then a falling edge occured
		if (!(pin & PCInt_RX_Pins[pin_pos]))
		{
			//calculate dt with current time subtracted by
			//the time of rising edge occured
			dTime = currTime - risingEdgeTime[pin_pos];
			if (900 < dTime && dTime < 2200)
			{
				rcRawValue[rc_value_pos] = dTime;
//add a little bit of failsafe routine if failsafe enabled
#if FAILSAFE == ENABLED
				//failsafe routine - what the fuck??
				if ((rc_value_pos == THROTTLEPIN || rc_value_pos == YAWPIN ||
				        rc_value_pos == PITCHPIN || rc_value_pos == ROLLPIN)
				        && dTime > FAILSAFE_DETECT_TRESHOLD)
					GoodPulses |= (1 << rc_value_pos);
#endif
			}
		}
		//if is high, then it is a rising edge,
		//store its start time in currTime;
		else
			risingEdgeTime[pin_pos] = currTime;
	}
}

/**
 * port change Interrupt.
 * this ISR is common to every receiver channel, it is
 * call everytime a change state occurs on a RX input pin.
 */
ISR(RX_PC_INTERRUPT) {
	uint8_t mask;
	uint8_t pin;
	uint16_t currTime; // current time time-stamp
	uint16_t dTime;
	//time stamp of each pin's rising edge stores here
	static uint16_t risingEdgeTime[8];
	static uint8_t PCintLast;

#if FAILSAFE == ENABLED
	static uint8_t GoodPulses;
#endif

	// RX_PCINT_PIN_PORT indicates the state of each PIN for
	// the arduino port dealing with Ports digital pins
	pin = RX_PCINT_PIN_PORT;
	// doing a ^ between the current interruption and
	// the last one indicates wich pin changed
	mask = pin ^ PCintLast;
	// micros() return a uint32_t, but it is not
	// useful to keep the whole bits => we keep only 16 bits
	currTime = (uint16_t) micros();
	// re enable other interrupts at this point,
	// the rest of this interrupt is not so time critical
	// and can be interrupted safely
	sei();
	// we memorize the current state of all PINs [D0-D7]
	PCintLast = pin;

	RX_PIN_CHECK(0, 2); 
	RX_PIN_CHECK(1, 4);
	RX_PIN_CHECK(2, 5);
	RX_PIN_CHECK(3, 6);
	RX_PIN_CHECK(4, 7);
/**
 * if wanna increase pin check, refer to the following syntax:
 * 
 *  #if (PCINT_PIN_COUNT > 5)
 *    RX_PIN_CHECK(5, 0);
 *  #endif
 */

#if defined(FAILSAFE)
	if (GoodPulses == (1 << THROTTLEPIN) + (1 << YAWPIN) + (1 << ROLLPIN) + (1 << PITCHPIN)) { // If all main four chanells have good pulses, clear FailSafe counter
		GoodPulses = 0;
		if (failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
	}
#endif
}


uint16_t readRawRC(uint8_t chan) {
	uint16_t data;

	uint8_t oldSREG;

	//store values inSREG
	oldSREG = SREG;
	//disable interrupts
	cli();
	//rcChannel[chan] => stores the mapping from channel to pin
	//e.g rcChannel[1] = ROLLPIN, ROLLPIN is pin 4 in PROMINI,
	//    so rcChannel[1]=4.
	//rcRawValue[] => stores each pin's raw PPM value
	data = rcRawValue[rcChannel[chan]];
	SREG = oldSREG;        // Let's restore interrupt state

	return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
#define AVERAGING_ARRAY_LENGTH 4
void computeRC() {
	static uint16_t rcData4Values[RC_CHANS][AVERAGING_ARRAY_LENGTH - 1];
	uint16_t rcDataMean, rcDataTmp;
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan, i;
	uint8_t failsafeGoodCondition = 1;

	//rcValuesIndex is used to implement something like a ring queue
	//for the moving average filter
	rc4ValuesIndex++;
	if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH - 1)
		rc4ValuesIndex = 0;

	for (chan = 0; chan < RC_CHANS; chan++)
	{
		rcDataTmp = readRawRC(chan);
#if FAILSAFE == ENABLED
		// update controls channel only if
		// pulse is above FAILSAFE_DETECT_TRESHOLD.
		// In disarmed state allow always update for easer configuration.
		failsafeGoodCondition =
		    rcDataTmp > FAILSAFE_DETECT_TRESHOLD || chan > 3 || !f.ARMED;
#endif
		if (failsafeGoodCondition)
		{
			//a simple moving average filter concerning
			//AVERAGING_ARRAY_LENGTH times of past data
			rcDataMean = rcDataTmp;
			for (i = 0; i < AVERAGING_ARRAY_LENGTH - 1; i++)
				rcDataMean += rcData4Values[chan][i];
			//the (AVERAGING_ARRAY_LENGTH / 2) is used for rounding to get a better precision
			//since the statement below is a integer division
			rcDataMean = (rcDataMean + (AVERAGING_ARRAY_LENGTH / 2)) / AVERAGING_ARRAY_LENGTH;

			//limit the increase rate of rcdata to a maxium of 2
			if ( rcDataMean < (uint16_t)rcData[chan] - 3)
				rcData[chan] = rcDataMean + 2;
			if ( rcDataMean > (uint16_t)rcData[chan] + 3)
				rcData[chan] = rcDataMean - 2;

			//update the moving average filter
			rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
		}
#if defined(FAILSAFE)
		failsafeCnt = 0;
#endif
	}
}
}