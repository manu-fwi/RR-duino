#ifndef RRDUINO_H
#define RRDUINO_H

#include "Arduino.h"

extern HardwareSerial& to_bus;
extern byte address;
extern bool save_cfg_to_eeprom;

// Defines for the protocol
#define START_BYTE 0xFF
#define TIME_OUT 100  // ms

// Define for the command bits positions
#define CMD_CFGCMD_BV 7
#define CMD_RWDIR_BV 4
#define CMD_SENS_TURN_BV 5
#define CMD_ALL_BV 6
#define CMD_CFG_SENS_TURN_BV 4
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

// DEBUG MACRO
#define DEBUG(msg) Serial.print(msg)
#define DEBUGLN(msg) Serial.println(msg)

#endif // RRDUINO_H
