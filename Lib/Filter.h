#ifndef FILTER_H
#define FILTER_H

#include "Sensors.h"
#include "arduino.h"

//global variable needed:dt

//a couple of filter types to choose from, only one could be chosen
/**
 * type No.1 is a complemetary filter based on Mahony's papers.
 * for more info look at <http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/>.
 */

/**
 * type No.2 is a complemetary filter based on Madgwick's paper. for more info
 * about paper look at <>, about its implementation look
 * at<http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/>.
 */

/**
 * type No.3 is a complemetary filter based on Mahony's paper. for more info
 * about paper look at <>. about its implementation look
 * at<>.
 */

/**
 * type No.4 is an Extended Kalman filter based on Kalman's 1960 paper
 * for more info about paper look at <>
 * for more info about implementation look at<>
 */


/**
 * =============parameters for filter No.1=============
 * need tune to balance between high-frequency responce and low-frequency
 * stability
 */
#define FILTER_TYPE_NO1_KP 4.7f
#define FILTER_TYPE_NO1_KI 0.0f

/**
 * =============parameters for filter No.2=============
 * need tune to balance between high-frequency responce and low-frequency
 * stability
 */
#define FILTER_TYPE_NO2_CONVERG_RATE 4.0f

class Filter
{
public:
	//constructor
	Filter();

	//members
	void update(float ax, float ay, float az,
	            float gx, float gy, float gz,
	            float mx, float my, float mz);

	//output attitude in euler angle mode
	void getEulerAngleDeg (
	    float* pitchWant, float* rollWant, float* yawWant);
private:

	/*quaternions used to store attitude*/
	volatile float q0, q1, q2, q3;
	/*euler angles(in deg) used to represent attitude*/
	volatile float pitch, roll, yaw;

	/*time stamp store here*/
	float currentT, lastT, dt;

	void typeNo1Update9DOF(float gx, float gy, float gz,
	                       float ax, float ay, float az,
	                       float mx, float my, float mz);
	void typeNo1Update6DOF(float gx, float gy, float gz,
	                       float ax, float ay, float az);

	void typeNo2Update9DOF(float gx, float gy, float gz,
	                       float ax, float ay, float az,
	                       float mx, float my, float mz);
	void typeNo2Update6DOF(float gx, float gy, float gz,
	                       float ax, float ay, float az);

	void typeNo4Update6DOF(float gx, float gy, float gz,
	                       float ax, float ay, float az);

	float invSqrt(float x);

};


#endif