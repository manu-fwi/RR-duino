#include "sensors.h"
#include "turnouts.h"
#include "RRduino.h"
#include <EEPROM.h>

sensor_cfg_t * sensor_cfg_head = NULL;

int eeprom_sensor_end = -1;   // address where to add new turnouts config, -1 means not calculated yet

unsigned sensors_chng_state=0;

// Find the last sensor with its subadd < subadd parameter
// This is to ensure that we add a new sensor so that the list of sensors stays in ascending order with respect to subadd

sensor_cfg_t * find_last_sensor_before(byte subadd)
{
  sensor_cfg_t * cur = sensor_cfg_head,* prev = NULL;
  while (cur && (cur->subadd<subadd)) {
    prev = cur;
    cur = cur->next;    
  }
  return prev;  
}

sensor_cfg_t * find_cfg_sensor(byte subadd)
{
  sensor_cfg_t * cur = sensor_cfg_head;
  while (cur) {
    if (cur->subadd == subadd)
      return cur;
    else if (cur->subadd>subadd)
      return NULL;
    cur = cur->next;
  }
  return NULL;
}

// Find the eeprom address of a sensor config from its subaddress
// if not found return the opposite of the addres after the last block
int ee_find_cfg_sensor(byte subadd)
{
  int ee_add=EEPROM.length()-CFG_SENSOR_SIZE;

  byte first = EEPROM.read(ee_add);
  while ((first&0x7F)!=(subadd&0x7F)) { // Check subadd whitout taking B7 (indicates I/O)
    if (first==0) {
      if (eeprom_sensor_end<0)
      // Set end of eeprom if not set already
        eeprom_sensor_end = ee_add;
      return -ee_add;
    }
    // Pass to the next config
    ee_add-=CFG_SENSOR_SIZE;
    first = EEPROM.read(ee_add);
  }
  return ee_add;
}

int ee_find_free_sensor()
{
  int ee_add=EEPROM.length()-CFG_SENSOR_SIZE;

  byte first = EEPROM.read(ee_add);
  while ((first!=0) && (first!=EE_FREE_SENSOR) && (ee_add>eeprom_sensor_end)) {
    ee_add -= CFG_SENSOR_SIZE;
    first = EEPROM.read(ee_add);
  }
  if ((ee_add==eeprom_sensor_end) && !room_in_eeprom(CFG_SENSOR_SIZE)) // No free slot, check if we can add one new turnout config
    return -1;
  else return ee_add;
}

sensor_cfg_t * read_cfg_sensor(int ee_add)  // read sensor_cfg_t struct in eeprom
{
  sensor_cfg_t * cfg = new sensor_cfg_t;
  if (!cfg)
    return NULL;
  cfg->subadd = EEPROM.read(ee_add++);
  cfg->sensor_pin = EEPROM.read(ee_add++);
  Serial.println(cfg->subadd,HEX);
  Serial.print(cfg->sensor_pin,HEX);
  Serial.println("*********");
  cfg->status = 1 << SENSOR_BV_SYNC;
  
  // Adjust status from eeprom content
  if (cfg->subadd & (1 << EE_SENSOR_SUB_IO_BV))
    cfg->status |= 1<< SENSOR_BV_IO;
  if (cfg->sensor_pin & (1 << EE_SENSOR_PIN_PULLUP_BV))
    cfg->status |= 1 << SENSOR_BV_PULLUP;
  if (cfg->subadd & (1 << EE_SENSOR_SUB_VALUE_BV))
    cfg->status |= 1;
  else
    cfg->status &= 0xFE;
  // Clear bits
  cfg->subadd &= 0x3F;  // Clean the subadd up
  cfg->sensor_pin &= 0x7F;
  cfg->last_time = 0;
  return cfg;
}

void save_cfg_sensor(int ee_add,sensor_cfg_t * sensor)
{
  byte i = sensor->subadd;
  // Save to eeprom: set bits in subadd and servo_pin according to status
  if (sensor->status & (1 << SENSOR_BV_IO))
    i |= (1<<EE_SENSOR_SUB_IO_BV);
      
  EEPROM.write(ee_add, i);
  i = sensor->sensor_pin;
  if (sensor->status & (1 << SENSOR_BV_PULLUP))
    i |= (1 << EE_SENSOR_PIN_PULLUP_BV);
  
  EEPROM.write(ee_add+1, i);
  // Mark as saved in eeprom
  sensor->status |= (1 << SENSOR_BV_SYNC);
  ee_add-=CFG_SENSOR_SIZE;
  if (ee_add<eeprom_sensor_end) {  
    eeprom_sensor_end=ee_add;
    EEPROM.write(eeprom_sensor_end,0);  // Marks the end of the sensors
  }
}

void update_cfg_sensor(byte subadd,byte pin,byte status)
{
  // Save to eeprom
  Serial.println("update sensor");
  int ee_add = ee_find_cfg_sensor(subadd);
  if (ee_add<0)
    return; // FIXME: error handling
    
  // set bits in subadd and servo_pin according to status
  if (status & (1 << SENSOR_BV_IO))
    subadd |= (1<<EE_SENSOR_SUB_IO_BV);
      
  EEPROM.write(ee_add, subadd);
  
  if (status & (1 << SENSOR_BV_PULLUP))
    pin |= (1<<EE_SENSOR_PIN_PULLUP_BV);
  EEPROM.update(ee_add+1, pin);
}

/* 
 * Make a string from the sensor struct: <AS subadd pin I/O/P>
 * and puts it in str (must be able to contain 17 chars + null byte ending
 */
 
void sensor_cfg_to_str(sensor_cfg_t * sens,char * str)
{
  snprintf(str,18,"<AS %d %d I>",sens->subadd,sens->sensor_pin);
  if (sens->status & (1<<SENSOR_BV_IO)) {
    if (sens->status & (1<<SENSOR_BV_PULLUP))
      str[strlen(str)-2]='P';
  } else str[strlen(str)-2]='O';
}

/* Poll for all input sensors
 * Could have used PCINT interrupts but these are not available on Leonardo
 * for example. So for now lets do it software only. Maybe later we'll use
 * interrupts for UNO
 */
bool check_all_sensors()
{
  sensor_cfg_t * current = sensor_cfg_head;
  while (current) {
    if (current->status & (1 << SENSOR_BV_IO)) {
      byte temp = (digitalRead(current->sensor_pin)==HIGH) ? 1 : 0;
     
      // Check if state changed
      if (temp!=((current->status >> SENSOR_BV_CHK_STATE)&0x01)) {
        //If yes update time stamp
        Serial.print("Changed Sensor:");
        Serial.print(current->subadd);
        Serial.print(" ");
        Serial.print(current->sensor_pin,HEX);
        Serial.print(" ");
        Serial.print(current->status,HEX);
        Serial.print(" ");
        Serial.print(sensors_chng_state);
        Serial.print(" ");
        Serial.println(current->last_time);
        current->last_time = millis();
        if (temp)
            current->status |= 1 << SENSOR_BV_CHK_STATE;
        else
          current->status &= ~(1 << SENSOR_BV_CHK_STATE);
      } else {
        // Same state, check if wa can validate it
        if (millis()>current->last_time+SENSOR_DEBOUNCE) {
          bool change = (current->status&0x01) ^temp;
          if (change) {
            current->status = (current->status & 0xFE)+temp; // Validate the new state
            current->last_time = millis();   // Reset time
            if ((current->status & (1<<SENSOR_BV_CHNG_STATE))==0)
            {
              current->status |= 1 << SENSOR_BV_CHNG_STATE;
              sensors_chng_state++;
            }
          }
        }
      }
    }
    current = current -> next;
  }
}
