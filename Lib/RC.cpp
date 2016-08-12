
#include "RC.h"

//RC computed and filterd value(ready to use) store here
static volatile uint16_t rcData[RC_CHANS] = {0, 0, 0, 0, 0, 0 , 0 , 0};

//RC raw values will be stored here
static volatile uint16_t rcRawValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

//channel to pin mapping will be stored here
static volatile uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN, AUX2PIN, AUX3PIN, AUX4PIN};

static volatile uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit

///////////////////////////////////////////////////////////
//      ISR starts here, used to catch PPM length        //
///////////////////////////////////////////////////////////

//a litte helper for ISR
#define RX_PIN_CHECK(pin_pos, rc_value_pos)           \
    if (mask & PCInt_RX_Pins[pin_pos]) {              \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {          \
        dTime = currentTime-risingEdgeTime[pin_pos];  \
        if (900<dTime && dTime<2200) {                \
          rcRawValue[rc_value_pos] = dTime;           \
        }                                             \
      } else risingEdgeTime[pin_pos] = currentTime;   \
    }

/**
 * port change Interrupt.
 * this ISR is common to every receiver channel, it is
 * call everytime a change state occurs on a RX input pin.
 */
ISR(RX_PC_INTERRUPT) {
	uint8_t mask;
	uint8_t pin;
	uint16_t currentTime; // current time time-stamp
	uint16_t dTime;
	//time stamp of each pin's rising edge stores here
	static uint16_t risingEdgeTime[8];
	static uint8_t PCintLast;

	// RX_PCINT_PIN_PORT indicates the state of each PIN for
	// the arduino port dealing with Ports digital pins
	pin = RX_PCINT_PIN_PORT;
	// doing a ^ between the current interruption and
	// the last one indicates wich pin changed
	mask = pin ^ PCintLast;
	// micros() return a uint32_t, but it is not
	// useful to keep the whole bits => we keep only 16 bits
	currentTime = (uint16_t) micros();
	// re enable other interrupts at this point,
	// the rest of this interrupt is not so time critical
	// and can be interrupted safely
	sei();
	// we memorize the current state of all PINs [D0-D7]
	PCintLast = pin;

	RX_PIN_CHECK(0, 2); //PCINT1, on pin SCK, raw data stores in rcRawValue[2]
	RX_PIN_CHECK(1, 4); //PCINT2, on pin MOSI, raw data stores in rcRawValue[4]
	RX_PIN_CHECK(2, 5); //PCINT3, on pin MISO, raw data stores in rcRawValue[5]
	RX_PIN_CHECK(3, 6); //PCINT4, on pin D8, raw data stores in rcRawValue[6]
//	RX_PIN_CHECK(4, 7);
	/**
	 * if wanna increase pin check, refer to the following syntax:
	 *
	 *  #if (PCINT_PIN_COUNT > 5)
	 *    RX_PIN_CHECK(5, 0);
	 *  #endif
	 */
}

// atmega32u4's Throttle throttle pin
ISR(INT6_vect) {
	static uint16_t now, diff;
	static uint16_t last = 0;
	now = micros();

	//if PE6(also pin D7 on Arduino leonardo) is low
	//then a falling edge occured, calculate dt
	if (!(PINE & (1 << 6)))
	{
		diff = now - last;
		if (900 < diff && diff < 2200)
		{
			rcRawValue[3] = diff;
#if defined(FAILSAFE)
			// if Throttle value is higher than
			// FAILSAFE_DETECT_TRESHOLD
			if (diff > FAILSAFE_DETECT_TRESHOLD)
				// If pulse present on THROTTLE pin
				// (independent from ardu version),
				// clear FailSafe counter  - added by MIS
				if (failsafeCnt > 20)
					failsafeCnt -= 20;
				else
					failsafeCnt = 0;
#endif
		}
	}
	//if PE6(also pin D7 on Arduino leonardo) is high
	//then a rising edge occured, record its time stamp
	else
		last = now;
}

///////////////////////////////////////////////////////////
//                  ISR ends here                        //
///////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
//           Here comes members of RC class              //
///////////////////////////////////////////////////////////

/**
 * default constructor
 */
RC::RC()
{
	;
}
void RC::printRCCSV()
{
	computeRC();

	for (uint8_t i = 0; i < RC_CHANS; i++)
	{
		Serial.print(rcData[i]);
		Serial.print(",");
	}
	//new line
	Serial.println("");
}

void RC::getRCValue(uint16_t* dataPtr, uint8_t number)
{
	for (uint8_t i = 0; i < number; ++i)
		*(dataPtr + i) = rcData[i];
}

/**
 * initialize timers, ports and interrupts
 */
void RC::initialize()
{
	/**
	 * initialize each rc pin for PCINT PCINT activation.
	 * for more info about PCINT, look at
	 * <http://www.51hei.com/bbs/dpj-47867-1.html>
	 * and Atmega32u4 datasheet
	 */
	for (uint8_t i = 0; i < PCINT_PIN_COUNT; i++) { // i think a for loop is ok for the init.
		PCINT_RX_PORT |= PCInt_RX_Pins[i];
		PCINT_RX_MASK |= PCInt_RX_Pins[i];
	}
	PCICR = PCIR_PORT_BIT;

	//Trottle on pin 7 (ATmega32U4)
	DDRE &= ~(1 << 6); // pin 7 to input
	PORTE |= (1 << 6); // enable pullups
	EICRB |= (1 << ISC60);
	EIMSK |= (1 << INT6); // enable interuppt

	/*========temporarliy do not support aux pins========*/
}

/**
 * compute and filter RC data
 * using a moving average filter
 */
void RC::computeRC()
{
#define AVERAGING_ARRAY_LENGTH 4

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

	/*reading sequence: ROLL, PITCH, YAW, THROTTLE, AUX1...*/
	for (chan = 0; chan < RC_CHANS; chan++)
	{
		rcDataTmp = readRawRC(chan);
#if defined(FAILSAFE)
		// update controls channel only if
		// pulse is above FAILSAFE_DETECT_TRESHOLD.
		// In disarmed state allow always update for easer configuration.
		failsafeGoodCondition =
		    //AUX channels(chan>3) are out of failsafe concerns
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

			//some kind of filter, if the difference between two
			//consecutive signal is less than -3 ~ +3, than the result value
			//remains the same
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

uint16_t RC::readRawRC(uint8_t chan)
{
	uint16_t data;

	uint8_t oldSREG;

	//store values inSREG
	oldSREG = SREG;
	//disable interrupts
	cli();
	//rcChannel[chan] => stores the mapping from channel to pin
	//e.g rcChannel[0] = ROLLPIN, ROLLPIN is pin 6 in PROMICRO,
	//    so rcChannel[0]=6.
	//rcRawValue[] => stores each pin's raw PPM value
	data = rcRawValue[rcChannel[chan]];
	SREG = oldSREG;        // Let's restore interrupt state

	return data; // We return the value correctly copied when the IRQ's where disabled
}













#if 0
/* AUX2 Init*/
// Aux2 pin on PBO (D17/RXLED)
#if defined(RCAUX2PIND17)
DDRB &= ~(1 << 0); // set D17 to input
#endif
// Aux2 pin on PD2 (RX0)
#if defined(RCAUX2PINRXO)
DDRD &= ~(1 << 2); // RX to input
PORTD |= (1 << 2); // enable pullups
EICRA |= (1 << ISC20);
EIMSK |= (1 << INT2); // enable interuppt
#endif

/* AUX2 ISR*/
// Aux 2
#if defined(RCAUX2PINRXO)
ISR(INT2_vect)
{
	static uint16_t now, diff;
	static uint16_t last = 0;
	now = micros();
	if (!(PIND & (1 << 2)))
	{
		diff = now - last;
		if (900 < diff && diff < 2200)
			rcRawValue[7] = diff;
	}
	else
		last = now;
}
#endif /* #if defined(RCAUX2PINRXO) */

#endif /* #if 0*/