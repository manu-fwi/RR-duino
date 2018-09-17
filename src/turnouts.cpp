#include "turnouts.h"
#include "sensors.h"
#include "RRduino.h"
#include <EEPROM.h>

turnout_cfg_t * turnout_cfg_head = NULL;
#ifdef TURNOUT_COMB
turn_comb_cfg_t * turn_comb_cfg_head = NULL;
#endif

int eeprom_turn_end = -1;   // address where to add new turnouts config, -1 means not calculated yet

// Find the last turnout with its subadd < subadd parameter
// This is to ensure that we add a new turnout so that the list of turnout stays in ascending order with respect to subadd
turnout_cfg_t * find_last_turn_before(byte subadd)
{
  turnout_cfg_t * cur = turnout_cfg_head,* prev = NULL;
  while (cur && (cur->subadd<subadd)) {
    prev = cur;
    cur = cur->next;    
  }
  return prev;  
}

turnout_cfg_t * find_cfg_turnout(byte subadd)
{
  turnout_cfg_t * cur = turnout_cfg_head;
  DEBUG("find cfg turnout ");
  DEBUGLN(subadd);
  while (cur) {
    DEBUGLN(cur->subadd);
    if (cur->subadd == subadd)
      return cur;
    /*else if (cur->subadd>subadd)
      return NULL;*/
    cur = cur->next;
  }
  return NULL;
}
turnout_cfg_t * find_cfg_turnout_by_pin(byte pin) // find the config in the configs list
{
  turnout_cfg_t * cur = turnout_cfg_head;
  while (cur)
    if (cur->servo_pin == pin)
      break;
    else cur = cur->next;
  return cur;  
  
}

#ifdef TURNOUT_COMB
turn_comb_cfg_t * find_cfg_turn_comb(byte subadds[4])
{
  turn_comb_cfg_t * cur = turn_comb_cfg_head;
  int i;
  while (cur) {
    for (i=0;i<4;i++)
      if (cur->subadds[i] != subadds[i])
        break;
    if (i==4)
      break;
    else cur = cur->next;
  }
  return cur;  
}
#endif

void ee_compute_turnout_end()
{
  int ee_add = EE_BEGIN_TURN;
  byte first = EEPROM.read(ee_add);
  while (first!=0) {
    ee_add += CFG_TURN_COMB_SIZE;
    first=EEPROM.read(ee_add);
  }
  eeprom_turn_end = ee_add;
  DEBUG("EEPROM turnout end=");
  DEBUGLN(eeprom_turn_end);
}

void ee_compute_sensor_end()
{
  int ee_add = EEPROM.length()-CFG_SENSOR_SIZE;
  byte first = EEPROM.read(ee_add);
  while (first!=0) {
    ee_add -= CFG_SENSOR_SIZE;
    first = EEPROM.read(ee_add);
  }
  eeprom_sensor_end = ee_add;
  DEBUG("EEPROM sensor end=");
  DEBUGLN(eeprom_sensor_end);  
}

bool room_in_eeprom(byte alloc_size)
{
  if (eeprom_turn_end==-1)
    ee_compute_turnout_end();
  if (eeprom_sensor_end==-1)
    ee_compute_sensor_end();

  DEBUG(eeprom_turn_end);
  DEBUG(" ");
  DEBUG(alloc_size);
  DEBUG(" ");
  DEBUG(eeprom_sensor_end);
  DEBUGLN();
  if (eeprom_turn_end+alloc_size+1<=eeprom_sensor_end)
    return true;
  else return false;
}

int ee_find_free_turnout()   // Find the first free slot or -1 if there is no room
{
  int ee_add = EE_BEGIN_TURN;

  byte first = EEPROM.read(ee_add);
  while ((first!=0) && (first!=EE_FREE_TURN) && (ee_add<eeprom_turn_end)) {
    ee_add += CFG_TURN_COMB_SIZE;
    first = EEPROM.read(ee_add);
  }
  if ((ee_add==eeprom_turn_end) && !room_in_eeprom(CFG_TURN_COMB_SIZE)) // No free slot, check if we can add one new turnout config
    return -1;
  else return ee_add;
}

// Find the eeprom address of a turnout config from its subaddress
// if not found return the opposite of the addres after the last block
int ee_find_cfg_turnout(byte subadd)
{
  int ee_add=EE_BEGIN_TURN;

  byte first = EEPROM.read(ee_add);
  while (first!=subadd) {
    if (first==0) {
      if (eeprom_turn_end<0)
      // Set end of eeprom if not set already
        eeprom_turn_end = ee_add;
      return -ee_add;
    }
    // Pass to the next config, we need to compute the size of the current config
    ee_add+=CFG_TURN_COMB_SIZE;
    first = EEPROM.read(ee_add);
  }
  return ee_add;
}

#ifdef TURNOUT_COMB
// Find the eeprom address of a turnout combination config from its subaddresses
// if not found return the opposite of the address after the last block
int ee_find_cfg_turn_comb(byte subadds[4])
{
  int ee_add=EE_BEGIN_TURN,i;
  bool found = false;
  
  while (!found)
  {
    int ee_temp = ee_add;
    if (EEPROM.read(ee_add)==0)
    {
      if (eeprom_turn_end<0)
      // Set end of eeprom if not set already
        eeprom_turn_end = ee_add;
      return -ee_add;
    }
    for (i=0;i<4;i++)
      if (EEPROM.read(ee_temp++)!=subadds[i])
        break;
    if (i==4)
      return ee_add;
        
    // Pass to the next config, we need to compute the size of the current config
    ee_add+=CFG_TURN_COMB_SIZE;
  }
  return ee_add;
}
#endif // TURNOUT_COMB

turnout_cfg_t * read_cfg_turn(int ee_add)  // read cfg_turnout_t struct in eeprom
{
  turnout_cfg_t * cfg = new turnout_cfg_t;
  if (!cfg)
    return NULL;
  cfg->subadd = EEPROM.read(ee_add++);
  byte servo_pin = EEPROM.read(ee_add++);
  cfg->servo_pin = servo_pin & 0x7F;
  cfg->straight_pos = EEPROM.read(ee_add++);
  cfg->thrown_pos = EEPROM.read(ee_add++);
  cfg->relay_pin_1 = EEPROM.read(ee_add++);
  cfg->relay_pin_2 = EEPROM.read(ee_add++);
  cfg->next = NULL;
  // Synced in eeprom, no servo attached, put it as moving to make sure it gets to the last known position
  cfg->status = (1<< TURNOUT_MOV_BV) | (1 << TURNOUT_SYNC_BV) | NO_SERVO;
  // Check last known pos (MSB of servo_pin)
  if (servo_pin & (1 << EE_TURNOUT_POS_BV))
    cfg->status |= (1 << TURNOUT_POS_BV);
  cfg->current_pos = UNVALID_POS;
  DEBUG(F("turnout read from eeprom:"));
  DEBUG(cfg->subadd);
  DEBUG(",");
  DEBUG(cfg->servo_pin);
  DEBUG(",");
  DEBUG(cfg->straight_pos);
  DEBUG(",");
  DEBUG(cfg->thrown_pos);
  DEBUG(",");
  DEBUG(cfg->relay_pin_1);
  DEBUG(",");
  DEBUGLN(cfg->relay_pin_2);
  
  return cfg;
}


#ifdef TURNOUT_COMB
turn_comb_cfg_t * read_cfg_comb(int ee_add)  // read turn_comb_cfg_t struct in eeprom
{
  // Allocate new config struct and populate it
  turn_comb_cfg_t * cfg = new turn_comb_cfg_t;
  if (!cfg)
    return NULL;
  int i=0;
  for (;i<4;i++)
    cfg->subadds[i] = EEPROM.read(ee_add++);
  for (i=0;i<2;i++)
    cfg->forbidden[i] = EEPROM.read(ee_add++);    
  cfg->next = NULL;
  cfg->saved |= 0x80;
}
#endif //TURNOUT_COMB

void update_cfg_turnout(turnout_cfg_t * cfg, int ee_add=-1) //update turnout cfg in eeprom
{
  if (ee_add<0)
    ee_add = ee_find_cfg_turnout(cfg->subadd);
  if (ee_add<0)
    return; // FIXME error handling
  ee_add++; // skip subadd
  byte servo_pin = cfg->servo_pin;
  if (cfg->status & (1 << TURNOUT_POS_BV))
    servo_pin |= (1 << EE_TURNOUT_POS_BV);
  EEPROM.update(ee_add++,servo_pin);
  EEPROM.update(ee_add++,cfg->straight_pos);
  EEPROM.update(ee_add++,cfg->thrown_pos);
  EEPROM.update(ee_add++,cfg->relay_pin_1);
  EEPROM.update(ee_add++,cfg->relay_pin_2);
  cfg->status |= (1<< TURNOUT_SYNC_BV);
}

void save_cfg_turnout(int ee_add,turnout_cfg_t * cfg)  // Save new cfg turnout
{
  byte servo_pin = cfg->servo_pin;
  if (cfg->status & (1 << TURNOUT_POS_BV))
    servo_pin |= (1 << EE_TURNOUT_POS_BV);
  EEPROM.update(ee_add++,cfg->subadd);
  EEPROM.update(ee_add++,servo_pin);
  EEPROM.update(ee_add++,cfg->straight_pos);
  EEPROM.update(ee_add++,cfg->thrown_pos);
  EEPROM.update(ee_add++,cfg->relay_pin_1);
  EEPROM.update(ee_add++,cfg->relay_pin_2);
  cfg->status |= (1<< TURNOUT_SYNC_BV);
  if (ee_add>eeprom_turn_end) {
    eeprom_turn_end = ee_add;
    EEPROM.update(ee_add,0); // Set the first byte of next block to 0 as it marks the end
  }
}

#ifdef TURNOUT_COMB
void update_cfg_turn_comb(turn_comb_cfg_t * cfg,int ee_add=-1) //update turnout cfg in eeprom
{
  if (ee_add<0)
    ee_add = ee_find_cfg_turn_comb(cfg->subadds);
  if (ee_add<0)
    return; // FIXME error handling
  ee_add+=4; // skip subadds
  for (int i=0;i<2;i++)
    EEPROM.update(ee_add++,cfg->forbidden[i]);
}

void save_cfg_turn_comb(int ee_add,turn_comb_cfg_t * cfg)  // Save new cfg turnout
{
  int i;
  for (i=0;i<4;i++)
    EEPROM.update(ee_add++,cfg->subadds[i]);
  for (i=0;i<2;i++)
    EEPROM.update(ee_add++,cfg->forbidden[i]);
  cfg->saved |= 0x80;
  if (ee_add>eeprom_turn_end) {
    EEPROM.update(ee_add,0); // Marks the end of the turnouts
    eeprom_turn_end = ee_add;
  }
}
#endif //TURNOUT_COMB
