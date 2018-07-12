#ifndef SENSORS_H
#define SENSORS_H
#include "Arduino.h"

/*********** Constants used by sensors*/

// Time top wait before validating sensor state
#define SENSOR_DEBOUNCE 20

/***************Structs**************/

#define SENSOR_BV_SYNC 7
#define SENSOR_BV_IO 6
#define SENSOR_BV_PULLUP 5

#define SENSOR_BV_CHK_STATE 1
#define SENSOR_BV_CHNG_STATE 2

#define EE_FREE_SENSOR 0x80

struct sensor_cfg_t
{
  byte subadd;      // subadd >= 1 as 0 is used to signal the end of the list in eeprom
  byte sensor_pin;
  /*
   *  B7=1 => sync in eeprom
   *  B6=I/0: 1=> Input / 0=> Output
   *  B5=1 => Pullup (if setup as Input)
   *  
   *  B2: state has changed
   *  B1: last checked state (need that for debounce)
   *  B0: last valid state
   */
  byte status;
  unsigned long last_time;  // Last time the state changed (used to debounce)
  sensor_cfg_t * next;
};

/*
 * EEPROM structure:
 * subadd (byte) : >=1  B7=1 => INPUT
 *                      B7=0 => OUTPUT
 *                      B6 = last known value (mostly used for output sensors)
 *                 Special value: EE_FREE_SENSOR (0x80) means deleted entry can be reused
 * sensor_pin (byte)    B7=0 => No PULLUP
 *                      B7=1 => PULLUP
 */

#define EE_SENSOR_SUB_IO_BV 7
#define EE_SENSOR_SUB_VALUE_BV 6
#define EE_SENSOR_PIN_PULLUP_BV 7

#define CFG_SENSOR_SIZE 2

/*********** Functions ********/

bool update_cfg_sensor();

sensor_cfg_t * find_last_sensor_before(byte subadd); // Find the last sensor with subaddress < subadd
sensor_cfg_t * find_cfg_sensor(byte subadd);  // find the config in the configs list
int ee_find_cfg_sensor(byte subadd);
int ee_find_free_sensor();
sensor_cfg_t * read_cfg_sensor(int ee_add);  // read cfg_sensor_t struct in eeprom
void save_cfg_sensor(int ee_add,sensor_cfg_t * sensor); // save cfg sensor in eeprom
void update_cfg_sensor(byte subadd,byte pin,byte status,int ee_add=-1); // update an existing config
void sensor_cfg_to_str(sensor_cfg_t * sens,char * str);

// check sensors: must be called on a regular basis (use the SENSOR_UPDATE timeout)
bool check_all_sensors();

extern int eeprom_sensor_end;

extern sensor_cfg_t * sensor_cfg_head;

extern unsigned sensors_chng_state;

#endif // SENSORS_H
