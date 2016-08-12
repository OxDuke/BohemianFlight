#ifndef RC_H
#define RC_H

#include "arduino.h"
#include "RCDefs.h"

/**
 * current support: seperate PPM mode
 * Will be added in the near future:
 *         > Serial sum PPM
 *         > SBUS(Futaba, Japan)
 *         > SPEKTRUM
 *         > DBUS(DJI, China)
 */

class RC
{
public:
	//constructor
	RC();

	void initialize();

	//print RC value of all channels in CSV format
	void printRCCSV();

	void getRCValue(uint16_t* dataPtr, uint8_t number);

private:

	void computeRC();

	//a helper function for RC::computeRC()
	uint16_t readRawRC(uint8_t chan);

};

#endif /* RC_H */