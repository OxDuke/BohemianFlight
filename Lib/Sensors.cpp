
#include "Wire.h"
#include "I2Cdev.h"

//include header files according to sensor types

#include "MPU6050.h"
#include "HMC5883L.h"
//#include "MS5611.h"
//#include "MS5803.h"

#include "Sensors.h"

/*define sensor instance*/
MPU6050 accelgyro;
HMC5883L mag;
//MS5611 baro;

/**
 * Default constructor
 */
Sensors::Sensors()
{
	ax = ay = az = 0;
	gx = gy = gz = 0;
	mx = my = mz = 0;
	alt = 0;

	axOffset = ayOffset = azOffset = 0;
	gxOffset = gyOffset = gzOffset = 0;
	mxOffset = myOffset = mzOffset = 0;
	altOffset = 0;
}

void Sensors::initialize()
{
	Wire.begin();
	accelgyro.initialize();
	mag.initialize();
//	baro.initialize();
}

void Sensors::calibarateAll()
{
	delay(1000);
	Serial.println("Calibrating sensors in 1 secs...");
	delay(1000);

	Serial.println("Calibrating ACC & Gyro...");

	float axCalib = 0.0f, ayCalib = 0.0f, azCalib = 0.0f;
	float gxCalib = 0.0f, gyCalib = 0.0f, gzCalib = 0.0f;

	//ACC Gyro calibrate
	//ACC Sensitivity: 16,384 LSB/g
	//Gyro Sensitivity: 16.4LSB/(º/s)
	for (uint16_t i = 0; i < 400 ; ++i)
	{
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		axCalib += ax;
		ayCalib += ay;
		azCalib += az;

		gxCalib += gx;
		gyCalib += gy;
		gzCalib += gz;
	}
//	axOffset = 1.0 * 16384 - (int16_t)(axCalib / 400);
//	ayOffset = 0 - (int16_t)(ayCalib / 400);
//	azOffset = 0 - (int16_t)(azCalib / 400);

	gxOffset = 0 - (int16_t)(gxCalib / 400);
	gyOffset = 0 - (int16_t)(gyCalib / 400);
	gzOffset = 0 - (int16_t)(gzCalib / 400);

	//MAG calibrate
	Serial.println("Calibrating MAG...");
	mxOffset = -180 ;
	myOffset = -21;
	mzOffset = 57;

	//Baro  calibrate
	Serial.println("Calibrating Baro...");
	altOffset = 0;

	Serial.println("Calibration Complete!");
	delay(1000);
}

void Sensors::readAll(
    int16_t* axOut, int16_t* ayOut, int16_t* azOut,
    float* gxOut, float* gyOut, float* gzOut,
    int16_t* mxOut, int16_t* myOut, int16_t* mzOut)
{
	updateAllRaw();

	*axOut = ax + axOffset;
	*ayOut = ay + ayOffset;
	*azOut = az + azOffset;

	/**
	 * transform to rad/s
	 * Gyro Sensitivity: 16.4LSB/(º/s) = 939.6508LSB/(rad/s)
	 */
	*gxOut = (float)(gx + gxOffset) / 939.6508;
	*gyOut = (float)(gy + gyOffset) / 939.6508;
	*gzOut = (float)(gz + gzOffset) / 939.6508;

	*mxOut = mx + mxOffset;
	*myOut = my + myOffset;
	*mzOut = mz + mzOffset;
}

void Sensors::readACCGYRO(
    int16_t* axOut, int16_t* ayOut, int16_t* azOut,
    float* gxOut, float* gyOut, float* gzOut)
{
	updateACCGYRORaw();

	*axOut = ax + axOffset;
	*ayOut = ay + ayOffset;
	*azOut = az + azOffset;

	*gxOut = (float)(gx + gxOffset) / 939.6508;
	*gyOut = (float)(gy + gyOffset) / 939.6508;
	*gzOut = (float)(gz + gzOffset) / 939.6508;
}

void Sensors::updateAllRaw()
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	mag.getHeading(&mx, &my, &mz);
//	baro.get;
}

void Sensors::updateACCGYROMAGRaw()
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	mag.getHeading(&mx, &my, &mz);
}
void Sensors::updateACCGYRORaw()
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void Sensors::updateMAGRaw()
{
	mag.getHeading(&mx, &my, &mz);
}

void Sensors::updateBaroRaw()
{
	;
}

void Sensors::printAllCSV()
{
	updateAllRaw();

	Serial.print("ACC:");
	Serial.print(ax + axOffset);
	Serial.print(",");
	Serial.print(ay + ayOffset);
	Serial.print(",");
	Serial.print(az + azOffset);
	Serial.print(" ");
	Serial.print("Gyro:");
	Serial.print(gx + gxOffset);
	Serial.print(",");
	Serial.print(gy + gyOffset);
	Serial.print(",");
	Serial.print(gz + gzOffset);
	Serial.print(" ");
	Serial.print("MAG:");
	Serial.print(mx + mxOffset);
	Serial.print(",");
	Serial.print(my + myOffset);
	Serial.print(",");
	Serial.print(mz + mzOffset);
	Serial.print(" ");
	Serial.print("Baro:");
//	Serial.print(_baro);
	Serial.println("");
}