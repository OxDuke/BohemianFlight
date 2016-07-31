
#include "Lib/Sensors.h"
#include "Lib/Filter.h"

float last = 0, current = 0, dt = 0;
bool blinkState = false;
int16_t ax, ay, az; 
int16_t mx=0, my=0, mz=0;
float gx, gy, gz;
float pitch, roll, yaw;

Sensors Sensors;
Filter Filter;

void setup() {
	Serial.begin(115200);
	last = micros();
	Sensors.initialize();
	Sensors.calibarateAll();
}
void loop() {

	/*calculaate dt*/
	current = (float)micros(); //save the timestamp right after reading
	dt = current - last;  //calculate the difference
	last = current;

	Sensors.readAll(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	Filter.update(ax, ay, az, gx, gy, gz, mx, my, mz);
	Filter.getEulerAngleDeg(&pitch, &roll, &yaw);

	Serial.print(pitch);
	Serial.print(",");
	Serial.print(roll);
	Serial.print(",");
	Serial.print(yaw);
	Serial.println("");
	
	blinkState = !blinkState;

}



