#ifndef RCDEFS_H
#define RCDEFS_H

/**
 * RC mapping table for Arduino Micro
 * 
 *    <1>  | <2> | <3>  |<4>|    <5>   |<6>
 * --------+-----+------+---+----------+---------
 *  PCINT1 | PB1 | SCK  | 2 | pitch    | 1
 *  PCINT2 | PB2 | MOSI | 4 | yaw      | 2
 *  PCINT3 | PB3 | MISO | 5 | aux1     | 4
 *  PCINT4 | PB4 | D8   | 6 | roll     | 0
 *  PCINT5 | PB5 | D9   | 7 | aux2     | 6 not used, pin D9 is using as PWM output
 *  INT6   | PE6 | D7   | 3 | throttle | 3
 *
 * <1> interruption name
 * <2> pin name on the Atmega32U4 chip
 * <3> pin name on the Ardu Microboard
 * <4> position in the array rcRawValue[]
 * <5> pin function
 * <6> channel number, position in array rcChannel[]
 * 
 */

#define RC_CHANS 8 //though only 5 channels in use

//defines the position of every channel in the raw value arrayrcRawValue[]
#define THROTTLEPIN   3
#define ROLLPIN       6
#define PITCHPIN      2
#define YAWPIN        4
#define AUX1PIN       5

#define AUX2PIN       7
#define AUX3PIN       1 // unused 
#define AUX4PIN       0 // unused 

#define PCINT_PIN_COUNT          4
#define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)

#define PCINT_RX_PORT                PORTB
#define PCINT_RX_MASK                PCMSK0
#define PCIR_PORT_BIT                (1<<0)
#define RX_PC_INTERRUPT              PCINT0_vect
#define RX_PCINT_PIN_PORT            PINB

#endif /* RCDEFS_H */