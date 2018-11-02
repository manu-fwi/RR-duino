#ifndef RRDUINO_H
#define RRDUINO_H

#include "Arduino.h"

extern HardwareSerial& to_bus;
extern byte data_dir_pin;  // Dir pin, 255 means normal serial
extern byte address;
extern bool save_cfg_to_eeprom;

// Defines for the protocol
#define START_BYTE 0xFF
#define TIME_OUT 100  // ms

// Define for the command bits positions
#define CMD_CFGCMD_BV 7
#define CMD_RWDIR_BV 5
#define CMD_ALL_BV 6
#define CMD_SENS_TURN_BV 4
#define CMD_CFG_DEL_BV 5
#define CMD_CFG_SPECIAL_BV 3
#define CMD_CFG_SPECIAL_EEPROM_BV 4
#define CMD_ASYNC_BV 2
#define CMD_PEND_ANSWERS_BV 1
#define CMD_CMD_ANSWER_BV 0

// Define for the address bits positions
#define ADD_LIST_BV 6

// Define for the subaddress bits positions
#define SUB_LAST_BV 7
#define SUB_IODIR_BV 6
#define SUB_VALUE_BV 6
#define SUB_RELAY_PIN_BV 6

// Define for the pin bits positions
#define PIN_PULLUP_BV 7
#define PIN_RELAY_PULSE_BV 7

// Error defines
#define EEPROM_FULL 1
#define MEMORY_FULL 2
#define UNKNOWN_DEV 3
#define UNVALID_DEV 4

// Other constants
#define MAX_CMD_LEN 63  // 64-1 we do not put the start byte into the buffer

// This pin must be held LOW to enable the "set address mode"
#define ADDRESS_MODE_PIN 2

void noop();
void set_data_dir(bool write=true);
// DEBUG MACRO
#define DEBUG(msg) Serial.print(msg)
#define DEBUGLN(msg) Serial.println(msg)
#define USE_DEBUG
//#define DEBUG(msg) noop()
//#define DEBUGLN(msg) noop()
//#undef USE_DEBUG

// Timer defines

// Counter limit => multiply by 10ms to know the pulse length (ex: 2 => 20ms pulse length)
#define TIMER_COUNT 2

#if defined(__AVR_ATmega32U4__)
//Use timer 4 for this one (Leonardo)
// Prescaler is set to 16384 so freq is 1KHz (period is 1ms)
// Use the OCR4C to increase  the period, here 10 so period is 10ms
#define USE_TIMER4

#define TIMER_ISR TIMER4_COMPA_vect

#define INIT_TIMER() ({\
  TCCR4B = 0x00;\
  TCCR4A = TCCR4C= TCCR4D = 0;\
  TCCR4B = (1<<CS43) | (1<<CS42)|(1<<CS41) | (1<<CS40);\
  TIMSK4 |= (1<<OCIE4A);\
  noInterrupts();\
  TC4H = 0;\
  OCR4C= 10;\
  interrupts();\
  })

#else
// Use timer 2 for others (Uno, Mega)
// Timer2 init counter
// 160 ticks
// The timer freq with 1024 prescaler is roughly 16000
// So this gets us a 100 Hz frequency (period is 10 ms)
#define USE_TIMER2

#define TIMER_ISR TIMER2_COMPA_vect


#define INIT_TIMER() ({\
  TCCR2B = 0x00;\
  TCCR2A = 0b00000010;\
  TCCR2B |= (1<<CS22)|(1<<CS21) | (1<<CS20);\
  TIMSK2 |= (1<<OCIE2A);\
  OCR2A= 160;\
})
#endif __AVR_ATmega32u4

#endif // RRDUINO_H
