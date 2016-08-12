
#ifndef PID_H
#define PID_H

#include "arduino.h"

class PID
{
public:
    //constuctor
    PID(const float& iniKp = 0.0f,
        const float& iniKi = 0.0f,
        const float& iniKd = 0.0f,
        const int16_t & iniImax = 0)
    {
        _Kp = iniKp; _Ki = iniKi; _Kd = iniKd; _Imax = iniImax;
        // set _lastDerivative as NotANumber when we startup
        _lastDerivative = NAN;
        _lastT == 0;
    };

    //get PID update
    float getPID(float error, float scaler);

    //reset I component
    void resetI();
    //reset I component & D component
    void resetID();

    float getKp() const {return _Kp;}
    float getKi() const {return _Ki;}
    float getKd() const {return _Kd;}
    int16_t getImax() const {return _Imax;}

    void setKp(const float newKp) {_Kp = newKp;}
    void setKi(const float newKi) {_Ki = newKi;}
    void setKd(const float newKd) {_Kd = newKd;}
    int16_t setImax(const int16_t newImax)
    {_Imax = abs(newImax);}
private:

    // P, I, D
    float _Kp;
    float _Ki;
    float _Kd;

    //maxium value of I component
    int16_t _Imax;

    float  _integrator; // I component stores here
    float _lastError;  //last error for derivative
    float _lastDerivative; // last derivative for low-pass filter
    uint32_t _lastT; // last time get_pid() was called in millis

    /**
     * Low pass filter cut frequency for derivative calculation.
     * 20 Hz becasue anything over that is probably noise, see
     * http://en.wikipedia.org/wiki/Low-pass_filter.
     */
    static const uint8_t        _fCut = 20;


};
#endif /* PID_H */
