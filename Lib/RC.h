#ifndef RC_H
#define RC_H

//different types of receivers
#define RC_MODE_PPM 0
#define RC_MODE_SERIAL_SUM_PPM 1
#define RC_MODE_SBUS 2
#define RC_MODE_SPEKTRUM 3
#define RC_MODE_DBUS 4

//choose a proper type of receiver mode
//default: RC_MODE_PPM, seperate PPM mode
#define RC_MODE RC_MODE_PPM


/**
 * current support: seperate PPM mode
 * Will be added in the near future:
 *         >Serial sum PPM
 *         >SBUS(Futaba, Japan)
 *         >SPEKTRUM
 *         >DBUS(DJI, China)
 */

class RC
{
public:
	//constructor
	RC();

	//print RC value of all channels in CSV format
	void printRCInCSV();

	void Initialize();

	void getValue();



private:
	uint16_t readRCRaw();
	inline void pinCheckState(uint8_t pin_pos, uint8_t rc_value_pos);

//RC raw values will be stored here
	volatile uint16_t rcRawValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

//channel to pin mapping will be stored here
	static uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN, AUX2PIN, AUX3PIN, AUX4PIN};
	static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit


};

#endif /* RC_H */