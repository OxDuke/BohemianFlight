
#include "Lib/Sensors.h"
#include "Lib/Filter.h"
#include "Lib/RC.h"
#include "Lib/PID.h"

//time variables
float last = 0, current = 0, dt = 0;

//imu variables
int16_t ax, ay, az;
int16_t mx = 0, my = 0, mz = 0;
float gx, gy, gz;
float pitch, roll, yaw;
float pitchRate, rollRate, yawRate;

#define WARP_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

//remote controller variables and defines
#define RC_CHANNELS_MAX 5
#define RC_ROLL 0
#define RC_PITCH 1
#define RC_YAW 2
#define RC_THROTTLE 3
#define RC_AUX1 4

uint16_t rcValue[RC_CHANNELS_MAX];
float targetPitch, targetRoll, targetYaw; //all in degrees
float targetThrottle;

#define RC_THROTTLE_MIN   1070
#define RC_THROTTLE_MAX   1970
#define RC_YAW_MIN   1068
#define RC_YAW_MAX   1915
#define RC_PITCH_MIN   1077
#define RC_PITCH_MAX   1915
#define RC_ROLL_MIN   1090
#define RC_ROLL_MAX   1913

#define RC_DEADBAND 30

//PID defines
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STABLE 2
#define PID_ROLL_STABLE 3
#define PID_YAW_RATE 4
#define PID_YAW_STABLE 5

// Motor pins defines
// because arduino use timer#0 to implement delay functions,
// so its PWM might not be very accurate
// so we use PWM pins on timers other than timer#0
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_RL   1    // rear left
#define MOTOR_RR   3    // rear right

Sensors Sensors;
Filter Filter;
RC RC;
PID pid[6];

void setup() {
	Serial.begin(115200);
	last = micros();

	allInitialize();
}
void loop() {

	/*calculaate dt*/
	current = (float)micros(); //save the timestamp right after reading
	dt = current - last;  //calculate the difference
	last = current;

delay(300);
	RC.printRCCSV();
//	RC.getRCValue(rcValue, RC_CHANNELS_MAX);
	//mapping
//	targetRoll = map(rcValue[RC_ROLL], RC_ROLL_MIN, RC_ROLL_MAX, -50, 50);
//	targetPitch = map(rcValue[RC_PITCH], RC_PITCH_MIN, RC_PITCH_MAX, -50, 50);
//	targetYaw = map(rcValue[RC_YAW], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
//	targetThrottle = map(rcValue[RC_THROTTLE], RC_THROTTLE_MIN, RC_THROTTLE_MAX, 0, 200);

	//cut deadband
	cutDeadband();

//	Sensors.readAll(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//	Sensors.printAllCSV();
//	Filter.update(ax, ay, az, gx, gy, gz, mx, my, mz);
//	Filter.getEulerAngleDeg(&pitch, &roll, &yaw);
//	printEulerAngleCSV();


//	need add a filter for gyrodata
//	rollRate=gx;
//	pitchRate=gy;
//	yawRate=gz;

//	calculatePIDAndOutput();
}

void allInitialize()
{
	//sensors initialize
	Sensors.initialize();
	Sensors.calibarateAll();

	//RC initialize
	RC.initialize();

	//pid initialize
	pid[PID_PITCH_RATE].setKp(0.7);
	pid[PID_PITCH_RATE].setKi(0.5);
	pid[PID_PITCH_RATE].setKd(0.3);
	pid[PID_PITCH_RATE].setImax(20);

	pid[PID_ROLL_RATE].setKp(0.7);
	pid[PID_ROLL_RATE].setKi(0.5);
	pid[PID_ROLL_RATE].setKd(0.3);
	pid[PID_ROLL_RATE].setImax(20);

	pid[PID_YAW_RATE].setKp(2.7);
	pid[PID_YAW_RATE].setKi(0.5);
	pid[PID_YAW_RATE].setKd(0.3);
	pid[PID_YAW_RATE].setImax(20);

	pid[PID_PITCH_STABLE].setKp(4.5);
	pid[PID_ROLL_STABLE].setKp(4.5);
	pid[PID_YAW_STABLE].setKp(10);

	//motor initialize
	pinMode(MOTOR_FL, OUTPUT);
	pinMode(MOTOR_FR, OUTPUT);
	pinMode(MOTOR_RL, OUTPUT);
	pinMode(MOTOR_RR, OUTPUT);

	analogWrite(MOTOR_FL, 0);
	analogWrite(MOTOR_FR, 0);
	analogWrite(MOTOR_RL, 0);
	analogWrite(MOTOR_RR, 0);
}

// RC map function
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cutDeadband()
{
	if (targetRoll >= 0.0f)
	{
		if (targetRoll < 0.2f)
			targetRoll = 0.0f;

		targetRoll = targetRoll - 0.2f;
	}
	else
	{
		if (targetRoll > -0.2f)
			targetRoll = 0.0f;

		targetRoll = targetRoll + 0.2f;
	}

	if (targetPitch >= 0.0f)
	{
		if (targetPitch < 0.2f)
			targetPitch = 0.0f;

		targetPitch = targetPitch - 0.2f;
	}
	else
	{
		if (targetPitch > -0.2f)
			targetPitch = 0.0f;

		targetPitch = targetPitch + 0.2f;
	}

	if (targetYaw >= 0.0f)
	{
		if (targetYaw < 0.2f)
			targetYaw = 0.0f;

		targetYaw = targetYaw - 0.2f;
	}
	else
	{
		if (targetYaw > -0.2f)
			targetYaw = 0.0f;

		targetYaw = targetYaw + 0.2f;
	}

	if (targetThrottle < 1.0f)
		targetThrottle = 0.0f;
	targetThrottle = targetThrottle - 1.0f;
}

void printEulerAngleCSV()
{
	Serial.print(dt);
	Serial.print(",");
	Serial.print(pitch);
	Serial.print(",");
	Serial.print(roll);
	Serial.print(",");
	Serial.print(yaw);
	Serial.println("");
}

void calculatePIDAndOutput()
{
	if (targetThrottle > 1.0f) //start stablelize
	{
		/*high level position PID*/
		float rollStableOutput = constrain(pid[PID_ROLL_STABLE].getPID(targetRoll - roll, 1), -250, 250);
		float pitchStableOutput = constrain(pid[PID_PITCH_STABLE].getPID(targetPitch - pitch, 1), -250, 250);
		float yawStableOutput = constrain(pid[PID_YAW_STABLE].getPID(WARP_180(targetYaw - yaw), 1), -360, 360);

		if (targetYaw > 0.0f) //yaw is rate driven
		{
			yawStableOutput = targetYaw;

		}

		/*low level rate PID*/
		float rollRateOutput = constrain(pid[PID_ROLL_RATE].getPID(rollStableOutput - rollRate, 1), -50, 50);
		float pitchRateOutput = constrain(pid[PID_PITCH_RATE].getPID(pitchStableOutput - pitchRate, 1), -50, 50);
		float yawRateOutput = constrain(pid[PID_YAW_RATE].getPID(yawStableOutput - yawRate, 1), -50, 50);

		//add up
		uint16_t motorFLOutput = targetThrottle + rollRateOutput + pitchRateOutput - yawRateOutput;
		uint16_t motorRLOutput = targetThrottle + rollRateOutput - pitchRateOutput + yawRateOutput;
		uint16_t motorFROutput = targetThrottle - rollRateOutput + pitchRateOutput - yawRateOutput;
		uint16_t motorRROutput = targetThrottle - rollRateOutput - pitchRateOutput - yawRateOutput;

		analogWrite(MOTOR_FL, motorFLOutput);
		analogWrite(MOTOR_FR, motorFROutput);
		analogWrite(MOTOR_RL, motorRLOutput);
		analogWrite(MOTOR_RR, motorRROutput);

	}
	else //turn off the engine
	{
		analogWrite(MOTOR_FL, 0);
		analogWrite(MOTOR_FR, 0);
		analogWrite(MOTOR_RL, 0);
		analogWrite(MOTOR_RR, 0);

		//reset PID I & D component
		for (int i = 0; i < 6; i++)
			pid[i].resetID();
	}
}
