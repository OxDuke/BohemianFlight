#ifndef SENSORS_H
#define SENSORS_H

#include "arduino.h"

class Sensors
{
public:
	//constructor
	Sensors();

	//initialize all sensors
	void initialize();

	void calibarateAll();

	/**
	 * read functions:
	 * output: ACC in corrected raw data
	 *         Gyro in corrected data in rad/s
	 *         MAG in corrected raw data
	 */
	void readAll(
	    int16_t* axOut, int16_t* ayOut, int16_t* azOut,
	    float* gxOut, float* gyOut, float* gzOut,
	    int16_t *mxOut, int16_t *myOut, int16_t *mzOut);
	void readACCGYRO(
	    int16_t* axOut, int16_t* ayOut, int16_t* azOut,
	    float* gxOut, float* gyOut, float* gzOut);

	//update all sensors at one time
	void updateAllRaw();
	//updateing a combination of sensors at one time
	/*ACC+GYRO+MAG*/
	void updateACCGYROMAGRaw();
	//updateing each kind of sensor seperately
	/*ACC only*/
	void updateACCGYRORaw();
	/*Gyro only*/
	void updateMAGRaw();
	/*Baro only*/
	void updateBaroRaw();

	//update and print all sensor raw data in CSV format,
	//used for integrity check
	void printAllCSV();

private:

	/*ACC raw value*/
	int16_t ax, ay, az;
	/*gyro raw value*/
	int16_t gx, gy, gz;
	/*MAG raw value*/
	int16_t mx, my, mz;
	/*Baro raw value*/
	int16_t alt;

	/*ACC Offset value*/
	int16_t axOffset, ayOffset, azOffset;
	/*gyro Offset value*/
	int16_t gxOffset, gyOffset, gzOffset;
	/*MAG Offset value*/
	int16_t mxOffset, myOffset, mzOffset;
	/*Baro Offset value*/
	int16_t altOffset;



};

#endif /* SENSORS_H */