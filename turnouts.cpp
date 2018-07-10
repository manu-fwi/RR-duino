#include "turnouts.h"
#include "RRduino.h"
#include <EEPROM.h>

turnout_cfg_t * turnout_cfg_head = NULL;
turn_comb_cfg_t * turn_comb_cfg_head = NULL;

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
  while (cur) {
    if (cur->subadd == subadd)
      return cur;
    else if (cur->subadd>subadd)
      return NULL;
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

bool room_in_eeprom(byte alloc_size)
{
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

turnout_cfg_t * read_cfg_turn(int ee_add)  // read cfg_turnout_t struct in eeprom
{
  turnout_cfg_t * cfg = new turnout_cfg_t;
  if (!cfg)
    return NULL;
  cfg->subadd = EEPROM.read(ee_add++);
  cfg->servo_pin = EEPROM.read(ee_add++);
  cfg->straight_pos = EEPROM.read(ee_add++);
  cfg->thrown_pos = EEPROM.read(ee_add++);
  cfg->relay_pin_1 = EEPROM.read(ee_add++);
  cfg->relay_pin_2 = EEPROM.read(ee_add++);
  cfg->next = NULL;
  cfg->status = 1 << 7 + NO_SERVO;
  cfg->current_pos = UNVALID_POS;
  return cfg;
}


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

void update_cfg_turnout(turnout_cfg_t * cfg, int ee_add=-1) //update turnout cfg in eeprom
{
  if (ee_add<0)
    ee_add = ee_find_cfg_turnout(cfg->subadd);
  if (ee_add<0)
    return; // FIXME error handling
  ee_add++; // skip subadd
  EEPROM.update(ee_add++,cfg->servo_pin);
  EEPROM.update(ee_add++,cfg->straight_pos);
  EEPROM.update(ee_add++,cfg->thrown_pos);
  EEPROM.update(ee_add++,cfg->relay_pin_1);
  EEPROM.update(ee_add++,cfg->relay_pin_2);
  Serial.println("update cfg_t");
}

void save_cfg_turnout(int ee_add,turnout_cfg_t * cfg)  // Save new cfg turnout
{
  EEPROM.update(ee_add++,cfg->subadd);
  EEPROM.update(ee_add++,cfg->servo_pin);
  EEPROM.update(ee_add++,cfg->straight_pos);
  EEPROM.update(ee_add++,cfg->thrown_pos);
  EEPROM.update(ee_add++,cfg->relay_pin_1);
  EEPROM.update(ee_add++,cfg->relay_pin_2);
  cfg->status |= 0x80;
  if (ee_add>eeprom_turn_end) {
    eeprom_turn_end = ee_add;
    EEPROM.update(ee_add,0); // Set the first byte of next block to 0 as it marks the end
  }
}

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

/*
 * Make a string from a turnout_cfg struct
 * str must be a buffer of at least 32 chars
 */
void turnout_cfg_to_str(turnout_cfg_t * cfg, char * str)
{
  if (cfg->relay_pin_1==0xFE)
    snprintf(str,32,"<AT %d %d %d %d>",cfg->subadd,cfg->servo_pin,cfg->straight_pos,cfg->thrown_pos);
  else
    snprintf(str,32,"<AT %d %d %d %d %d %d>",cfg->subadd,cfg->servo_pin,cfg->straight_pos,cfg->thrown_pos,
             cfg->relay_pin_1,cfg->relay_pin_2);
}

static byte nb_turnouts_cfg(turn_comb_cfg_t * cfg)
{
  byte i =0;
  while ((cfg->subadds[i]&0x7F)!=0) i++;
  return i;
}

static byte nibble(byte number,byte which_nibble)
{
  if (which_nibble==1)
    number >>= 4;
  return number & 0x0F;
}
/*
 * Make a string from a turnout_cfg struct
 * str must be a buffer of at least 44 chars
 */

void turn_comb_cfg_to_str(turn_comb_cfg_t * cfg, char * str)
{
  byte nb_turn = nb_turnouts_cfg(cfg);
  byte pos = snprintf(str,44,"<AC %d ",nb_turn);
  int i;
  for (i=0;i<nb_turn;i++)
    pos += snprintf(str+pos,44-pos,"%d ",cfg->subadds[i]&0x7F);
  byte j=0, nib=nibble(cfg->forbidden[0],0);
  pos--;   // back to the last space
  do {
    str[pos++]=' ';
    for (i=nb_turn-1;i>=0;i--) {
      str[pos++]='0'+((nib>>i)&0x01);
    }
    j++;
    nib = nibble(cfg->forbidden[j/2],j%2);
  } while ((nib!=0) && (j<4));
  str[pos++]='>';
  str[pos]='\0';
}
