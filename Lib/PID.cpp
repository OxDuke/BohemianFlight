
#include "PID.h"


float PID::getPID(float error, float scaler)
{
  uint32_t  currentT = micros(); //in microseconds
  uint32_t dt = currentT - _lastT; //dt in microseconds

  float output = 0.0f;
  float deltaTime = 0.0f;
  float derivative;

  /**
   * if this PID hasn't been used for a full second
   * or this is the first time this PID is used
   */
  if (dt > 1000000 || _lastT == 0)
  {
    dt = 0;
    // if this PID hasn't been used for a full second then zero
    // the intergator term. This prevents I buildup from a
    // previous fight mode from causing a massive return before
    // the integrator gets a chance to correct itself
    resetID();
  }

  _lastT = currentT;

  //deltaTime in seconds
  deltaTime = (float)dt / 1000000.0f;

  /*==== P component====*/
  if (abs(_Kp) > 0)
    output += error * _Kp;

  /*==== I component====*/
  if (abs(_Ki) > 0)
  {
    _integrator += error * _Ki * dt;
    if (_integrator < -_Imax)
      _integrator = -_Imax;
    if (_integrator > _Imax)
      _integrator = _Imax;
    output += _integrator;
  }

  /*==== D component====*/
  if (_lastDerivative == NAN)
  {
    // we've just done a reset, suppress the first derivative
    // term as we don't want a sudden change in input to cause
    // a large D output change
    derivative = 0;
    _lastDerivative = 0;
  }
  else
    derivative = (error - _lastError) / deltaTime;

  // discrete low pass filter, cuts out the
  // high frequency noise that can drive the controller crazy
  float RC = 1 / (2 * PI * _fCut);
  derivative = _lastDerivative +
               ((deltaTime / (RC + deltaTime)) *
                (derivative - _lastDerivative));

  // update state
  _lastError = error;
  _lastDerivative = derivative;

  // add in derivative component
  output += _Kd * derivative;

  /*==== scaler multiplication====*/
  output *= scaler;

  return output;
}

void PID::resetI()
{
  _integrator = 0;
}

void PID::resetID()
{
  _integrator = 0;
  // we use NaN (Not A Number) to indicate that the last
  // derivative value is not valid
  _lastDerivative = NAN;
}





#if 0
void annexCode()
{ // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,
  // depending on throttle value

#define DYN_THR_PID_CHANNEL THROTTLE

  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[DYN_THR_PID_CHANNEL] > 1500) { // breakpoint is fix: 1500
    if (rcData[DYN_THR_PID_CHANNEL] < 2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID * (rcData[DYN_THR_PID_CHANNEL] - 1500) >> 9); //  /512 instead of /500
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for (axis = 0; axis < 3; axis++) {
    tmp = min(abs(rcData[axis] - MIDRC), 500);
#if defined(DEADBAND)
    if (tmp > DEADBAND) { tmp -= DEADBAND; }
    else { tmp = 0; }
#endif
    if (axis != 2) { //ROLL & PITCH
      tmp2 = tmp >> 7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp - (tmp2 << 7)) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) >> 7);
      prop1 = 128 - ((uint16_t)conf.rollPitchRate * tmp >> 9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1 * prop2 >> 7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8 * prop1 >> 7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8 * prop1 >> 7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  tmp = (uint32_t)(tmp - MINCHECK) * 2559 / (2000 - MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp / 256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 256) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]
#if defined(HEADFREE)
  if (f.HEADFREE_MODE) { //to optimize
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }
#endif

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader = 0;
  switch (analogReader++ % (3 + VBAT_CELLS_NUM)) {
  case 0:
  {
#if defined(POWERMETER_HARD)
    static uint32_t lastRead = currentTime;
    static uint8_t ind = 0;
    static uint16_t pvec[PSENSOR_SMOOTH], psum;
    uint16_t p =  analogRead(PSENSORPIN);
    //LCDprintInt16(p); LCDcrlf();
    //debug[0] = p;
#if PSENSOR_SMOOTH != 1
    psum += p;
    psum -= pvec[ind];
    pvec[ind++] = p;
    ind %= PSENSOR_SMOOTH;
    p = psum / PSENSOR_SMOOTH;
#endif
    powerValue = ( conf.psensornull > p ? conf.psensornull - p : p - conf.psensornull); // do not use abs(), it would induce implicit cast to uint and overrun
    analog.amperage = ((uint32_t)powerValue * conf.pint2ma) / 100; // [100mA]    //old (will overflow for 65A: powerValue * conf.pint2ma; // [1mA]
    pMeter[PMOTOR_SUM] += ((currentTime - lastRead) * (uint32_t)((uint32_t)powerValue * conf.pint2ma)) / 100000; // [10 mA * msec]
    lastRead = currentTime;
#endif // POWERMETER_HARD
    break;
  }

  case 1:
  {
#if defined(VBAT) && !defined(VBAT_CELLS)
    static uint8_t ind = 0;
    static uint16_t vvec[VBAT_SMOOTH], vsum;
    uint16_t v = analogRead(V_BATPIN);
#if VBAT_SMOOTH == 1
    analog.vbat = (v * VBAT_PRESCALER) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#else
    vsum += v;
    vsum -= vvec[ind];
    vvec[ind++] = v;
    ind %= VBAT_SMOOTH;
#if VBAT_SMOOTH == VBAT_PRESCALER
    analog.vbat = vsum / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#elif VBAT_SMOOTH < VBAT_PRESCALER
    analog.vbat = (vsum * (VBAT_PRESCALER / VBAT_SMOOTH)) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#else
    analog.vbat = ((vsum / VBAT_SMOOTH) * VBAT_PRESCALER) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#endif
#endif
#endif // VBAT
    break;
  }
  case 2:
  {
#if defined(RX_RSSI)
    static uint8_t ind = 0;
    static uint16_t rvec[RSSI_SMOOTH], rsum, r;

    // http://www.multiwii.com/forum/viewtopic.php?f=8&t=5530
#if defined(RX_RSSI_CHAN)
    uint16_t rssi_Input = constrain(rcData[RX_RSSI_CHAN], 1000, 2000);
    r = map((uint16_t)rssi_Input , 1000, 2000, 0, 1023);
#else
    r = analogRead(RX_RSSI_PIN);
#endif

#if RSSI_SMOOTH == 1
    analog.rssi = r;
#else
    rsum += r;
    rsum -= rvec[ind];
    rvec[ind++] = r;
    ind %= RSSI_SMOOTH;
    r = rsum / RSSI_SMOOTH;
    analog.rssi = r;
#endif
#endif // RX RSSI
    break;
  }
  default: // here analogReader >=4, because of ++ in switch()
  {
#if defined(VBAT) && defined(VBAT_CELLS)
    if ( (analogReader < 4) || (analogReader > 4 + VBAT_CELLS_NUM - 1) ) break;
    uint8_t ind = analogReader - 4;
    static uint16_t vbatcells_pins[VBAT_CELLS_NUM] = VBAT_CELLS_PINS;
    static uint8_t  vbatcells_offset[VBAT_CELLS_NUM] = VBAT_CELLS_OFFSETS;
    static uint8_t  vbatcells_div[VBAT_CELLS_NUM] = VBAT_CELLS_DIVS;
    uint16_t v = analogRead(vbatcells_pins[ind]);
    analog.vbatcells[ind] = vbatcells_offset[ind] + (v << 2) / vbatcells_div[ind]; // result is Vbatt in 0.1V steps
    if (ind == VBAT_CELLS_NUM - 1) analog.vbat = analog.vbatcells[ind];
#endif // VBAT) && defined(VBAT_CELLS)
    break;
  } // end default
  } // end of switch()

#if defined( POWERMETER_HARD ) && (defined(LOG_VALUES) || defined(LCD_TELEMETRY))
  if (analog.amperage > powerValueMaxMAH) powerValueMaxMAH = analog.amperage;
#endif

#if defined(WATTS)
  analog.watts = (analog.amperage * analog.vbat) / 100; // [0.1A] * [0.1V] / 100 = [Watt]
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  if (analog.watts > wattsMax) wattsMax = analog.watts;
#endif
#endif

#if defined(BUZZER)
  alarmHandler(); // external buzzer routine that handles buzzer events globally now
#endif


  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

#if defined(LED_RING)
  static uint32_t LEDTime;
  if ( currentTime > LEDTime ) {
    LEDTime = currentTime + 50000;
    i2CLedRingState();
  }
#endif

#if defined(LED_FLASHER)
  auto_switch_led_flasher();
#endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

#if !(defined(SERIAL_RX) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if SERIAL RX in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
  serialCom();
#endif

#if defined(POWERMETER)
  analog.intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
  intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE;
#endif

#ifdef LCD_TELEMETRY_AUTO
  static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
  static uint8_t telemetryAutoIndex = 0;
  static uint16_t telemetryAutoTimer = 0;
  if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ) {
    telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
    LCDclear(); // make sure to clear away remnants
  }
#endif
#ifdef LCD_TELEMETRY
  static uint16_t telemetryTimer = 0;
  if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
#if (LCD_TELEMETRY_DEBUG+0 > 0)
    telemetry = LCD_TELEMETRY_DEBUG;
#endif
    if (telemetry) lcd_telemetry();
  }
#endif

#ifdef TELEMETRY
  run_telemetry();
#endif

#if GPS & defined(GPS_LED_INDICATOR)       // modified by MIS to use STABLEPIN LED for number of sattelites indication
  static uint32_t GPSLEDTime;              // - No GPS FIX -> LED blink at speed of incoming GPS frames
  static uint8_t blcnt;                    // - Fix and sat no. bellow 5 -> LED off
  if (currentTime > GPSLEDTime) {          // - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ...
    if (f.GPS_FIX && GPS_numSat >= 5) {
      if (++blcnt > 2 * GPS_numSat) blcnt = 0;
      GPSLEDTime = currentTime + 150000;
      if (blcnt >= 10 && ((blcnt % 2) == 0)) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
    } else {
      if ((GPS_update == 1) && !f.GPS_FIX) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
      blcnt = 0;
    }
  }
#endif

#if defined(LOG_VALUES) && (LOG_VALUES >= 2)
  if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
  if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
#endif
  if (f.ARMED)  {
#if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT) || defined (TELEMETRY)
    armedTime += (uint32_t)cycleTime;
#endif
#if defined(VBAT)
    if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
#endif
#ifdef LCD_TELEMETRY
#if BARO
    if ( (alt.EstAlt > BAROaltMax) ) BAROaltMax = alt.EstAlt;
#endif
#if GPS
    if ( (GPS_speed > GPS_speedMax) ) GPS_speedMax = GPS_speed;
#endif
#endif
  }
}





float PID::get_pid(float error, float scaler)
{
  uint32_t tnow = hal.scheduler->millis();
  uint32_t dt = tnow - _last_t;
  float output            = 0;
  float delta_time;

  if (_last_t == 0 || dt > 1000) {
    dt = 0;

    // if this PID hasn't been used for a full second then zero
    // the intergator term. This prevents I buildup from a
    // previous fight mode from causing a massive return before
    // the integrator gets a chance to correct itself
    reset_I();
  }
  _last_t = tnow;

  delta_time = (float)dt / 1000.0f;

  // Compute proportional component
  output += error * _kp;

  // Compute derivative component if time has elapsed
  if ((fabsf(_kd) > 0) && (dt > 0)) {
    float derivative;

    if (isnan(_last_derivative)) {
      // we've just done a reset, suppress the first derivative
      // term as we don't want a sudden change in input to cause
      // a large D output change
      derivative = 0;
      _last_derivative = 0;
    } else {
      derivative = (error - _last_error) / delta_time;
    }

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    float RC = 1 / (2 * PI * _fCut);
    derivative = _last_derivative +
                 ((delta_time / (RC + delta_time)) *
                  (derivative - _last_derivative));

    // update state
    _last_error             = error;
    _last_derivative    = derivative;

    // add in derivative component
    output                          += _kd * derivative;
  }

  // scale the P and D components
  output *= scaler;

  // Compute integral component if time has elapsed
  if ((fabsf(_ki) > 0) && (dt > 0)) {
    _integrator             += (error * _ki) * scaler * delta_time;
    if (_integrator < -_imax) {
      _integrator = -_imax;
    } else if (_integrator > _imax) {
      _integrator = _imax;
    }
    output                          += _integrator;
  }

  return output;
}

#endif