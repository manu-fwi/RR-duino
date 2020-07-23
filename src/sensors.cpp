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
  while ((first&0x3F)!=(subadd&0x3F)) { // Check subadd whitout taking B7 (indicates I/O) and B6 (value)
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
  if ((ee_add==eeprom_sensor_end) && !room_in_eeprom(CFG_SENSOR_SIZE)) // No free slot
    return -1;
  else return ee_add;
}

sensor_cfg_t * read_cfg_sensor(int ee_add)  // read sensor_cfg_t struct in eeprom
{
  sensor_cfg_t * cfg = new sensor_cfg_t;
  if (!cfg)
    return NULL;
  byte subadd = EEPROM.read(ee_add++);
  if (subadd==EE_FREE_SENSOR) {
  // Non sensor here it has been deleted
    cfg->subadd = EE_FREE_SENSOR;
    return cfg;
  }
  cfg->subadd = subadd & 0x3F;
  byte pin = EEPROM.read(ee_add++);
  cfg->sensor_pin = pin & 0x7F;
  DEBUG("sensor read from eeprom:");
  DEBUG(cfg->subadd);
  DEBUG(",");
  DEBUGLN(cfg->sensor_pin);
  cfg->status = 1 << SENSOR_SYNC_BV;
  
  // Adjust status from eeprom content
  if (subadd & (1 << EE_SENSOR_SUB_IO_BV))
    cfg->status |= 1<< SENSOR_IO_BV;
  if (pin & (1 << EE_SENSOR_PIN_PULLUP_BV))
    cfg->status |= 1 << SENSOR_PULLUP_BV;
  if (subadd & (1 << EE_SENSOR_SUB_VALUE_BV))
    cfg->status |= 1;

  cfg->last_time = millis();
  return cfg;
}

void save_cfg_sensor(int ee_add,sensor_cfg_t * sensor)
{
  DEBUG(F("Save sensor "));
  DEBUG(eeprom_sensor_end);
  DEBUG(" ");
  DEBUG(ee_add);
  DEBUG(" ");
  byte subadd = sensor->subadd;
  byte pin = sensor->sensor_pin;
  // Save to eeprom: set bits in subadd and servo_pin according to status
  if (sensor->status & (1 << SENSOR_IO_BV)) {
    subadd |= (1<<EE_SENSOR_SUB_IO_BV);
    if (sensor->status & (1 << SENSOR_PULLUP_BV))
      pin |= (1 << EE_SENSOR_PIN_PULLUP_BV);
  }
  if (sensor->status & 1)
    subadd |= (1 << EE_SENSOR_SUB_VALUE_BV);
    
  EEPROM.update(ee_add, subadd);
  EEPROM.update(ee_add+1,pin);
  // Mark as saved in eeprom
  sensor->status |= (1 << SENSOR_SYNC_BV);
  ee_add-=CFG_SENSOR_SIZE;
  if (ee_add<eeprom_sensor_end) {  
    eeprom_sensor_end=ee_add;
    EEPROM.update(eeprom_sensor_end,0);  // Marks the end of the sensors
  }
  DEBUG(subadd);
  DEBUG(" ");
  DEBUGLN(pin);
  DEBUGLN(eeprom_sensor_end);
}

void update_cfg_sensor(byte subadd,byte pin,byte status,int ee_add=-1)
{
  // Save to eeprom
  DEBUGLN(F("update sensor"));
  if (ee_add<0)
    ee_add = ee_find_cfg_sensor(subadd);
  if (ee_add<0)
    return; // FIXME: error handling
    
  // set bits in subadd and servo_pin according to status
  if (status & (1 << SENSOR_IO_BV))
    subadd |= (1<<EE_SENSOR_SUB_IO_BV);
      
  if (status & 1)
    subadd |= (1 << EE_SENSOR_SUB_VALUE_BV);

  EEPROM.update(ee_add, subadd);
  
  if (status & (1 << SENSOR_PULLUP_BV))
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
  if (sens->status & (1<<SENSOR_IO_BV)) {
    if (sens->status & (1<<SENSOR_PULLUP_BV))
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
    if (current->status & (1 << SENSOR_IO_BV)) {
      byte temp = (digitalRead(current->sensor_pin)==HIGH) ? 1 : 0;
     
      // Check if state changed
      if (temp!=((current->status >> SENSOR_CHK_STATE_BV)&0x01)) {
        //If yes update time stamp
        current->last_time = millis();
        // And state
        if (temp)
            current->status |= (1 << SENSOR_CHK_STATE_BV);
        else
          current->status &= ~(1 << SENSOR_CHK_STATE_BV);
      } else {
        // Same state, check if we can validate it
        if (current->last_time && (millis()-current->last_time>SENSOR_DEBOUNCE)) {
          byte change = (current->status&0x01) ^ temp;
          DEBUG(F("CHANGING STABILIZED "));
          DEBUGLN(current->status&0x01);
          DEBUGLN(temp);
          DEBUGLN(change);
          current->last_time = 0;   // No need to go on detecting after now
          if (change) {
            current->status = (current->status & 0xFE)+temp; // Validate the new state
            //FIXME
             DEBUG(F("CHNG STATE="));
            DEBUGLN(current->status & (1<<SENSOR_CHNG_STATE_BV));
            if ((current->status & (1<<SENSOR_CHNG_STATE_BV))==0)
            {
              current->status |= (1 << SENSOR_CHNG_STATE_BV);
              sensors_chng_state++;
              current->last_time = 0; // Special value that says we have detected a change in the state
              // No need to go on
            }
          }
        }
      }
    }
    current = current -> next;
  }
}
