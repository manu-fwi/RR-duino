#ifndef TURNOUT_H
#define TURNOUT_H
#include "Arduino.h"

//No turnout combination for now
#undef TURNOUT_COMB

#define NB_SERVOS 5     // for uno you can go up to 12, for mega up to 48 (make sure you have the power supply to go with that! Do not use the mega's one!)
                        // MAX is 63 (including the one reserved for fine tuning)
#define NO_SERVO 0b00011111

#define EE_BEGIN_TURN 2

#define EE_FREE_TURN 0b10000000

#define UNVALID_POS 0xFF   // Indicates that the turnout has not been positioned at all

#define TURNOUT_SYNC_BV 7
#define TURNOUT_MOV_BV 6
#define TURNOUT_POS_BV 5

#define RELAY_PULSE_LEN 10  // 10ms pulse for latching relays

#define REPOS_TURNOUT_TIME 120 // in ms for reposition time for servos

struct turnout_cfg_t
{
  byte subadd;    // B7: reserved B7:0 (turnout config)
                  // subadd >=1 as 0 is used to signal the end of the list in eeprom
                  // Reserved value: EE_FREE_TURN : means free slot (this is used when a turnout is deleted
  byte servo_pin;
  byte straight_pos;
  byte thrown_pos;
  byte current_pos; // turnout pos
  byte relay_pin_1; // activated when set to straight (=254 if unneeded)  (B7 = 1 <=> relay pin must be pulsed)
  byte relay_pin_2; // activated when set to thrown (=254 if unneeded)    (B7 = 1 <=> relay pin must be pulsed)
  /*
   *  B7=1 => synced in eeprom
   *  B6=0: not moving =1:moving
   *  B5=1 => in thrown position or moving from straight to thrown
   *  B5=0 => in straight position or moving from thrown to straight
   *  B0-B4: num of the servo assigned to this turnout: 0b11111 => not assigned
   */
  byte status;

  turnout_cfg_t * next;
};

#define EE_TURNOUT_POS_BV 7
/*
 * EEPROM structure:
 * subadd (byte) : B7: 0(=turnout) 1 is for forbidden combinations see below
 * servo_pin (byte)  : B7 = last known value
 * straight_pos (byte)
 * thrown_pos (byte)
 * relay pin 1 (byte) 254 if not set (B7 = 1 <=> relay pin must be pulsed)
 * relay pin 2 (byte) 254 if not set (B7 = 1 <=> relay pin must be pulsed)
 */

#ifdef TURNOUT_COMB
struct turn_comb_cfg_t
{
  byte subadds[4];   // subadds of the turnouts (in ascending order) (max 4) 0 for last subadds if you want less than 4
                     // subadds[0][B7]==1 for forbidden combinations
                     // Reserved value for subadds[0]: EE_FREE_TURN : means free slot (this is used when a turnout is deleted
  byte forbidden[2]; // forbidden combinations: each nibble contains a forbidden combination so max is 4
                     // Moreover only the first LSB nibble can be 0000 (indicating that the turnout cannot be all in straight positions
                     // Any other nibble being 0000 are then treated as unused
                     // (LSB first, first turnout pos is in least significant bit)
                     // example: for a 3-way turnout, 2 subadds then 0 in subadds[2] and subadds[0][B7]=1
                     // forbidden[0] is 0x0000 0011 if they can not be thrown at the same time (highest weight nibble is 0000 -> unused
                     // forbidden[1] is 0x0000 0000 as there is only one forbidden combination
  byte saved;        // B7=1=> sync in eeprom
  turn_comb_cfg_t * next;
};

/*
 * EEPROM structure:
 * subadds (byte[4])  subadds[0][B7]==1 and subadds[1][B7]==1 if 2 forbidden combinations.
 * forbidden (byte[2])
 */
 
/*
 * Note: if a turnout or combination is deleted, you mark the corresponding entry as invalid
 * by putting subadd to 0x80 (subadd==0 is not valid), not zero as this marks the end of the entries list
 */

turn_comb_cfg_t * find_cfg_turn_comb(byte subadd[4]);

// EEPROM related functions
int ee_find_cfg_turn_comb(byte subadds[4]);
turn_comb_cfg_t * read_cfg_comb(int ee_add);  // read cfg_turn_comb_t struct in 
void update_cfg_turn_comb(turn_comb_cfg_t * cfg,int ee_add=-1); //update turnout comb cfg in eeprom
void save_cfg_turn_comb(int ee_add,turn_comb_cfg_t * cfg); //save turnout comb cfg in eeprom at address ee_add

extern turn_comb_cfg_t * turn_comb_cfg_head;

#endif //TURNOUT_COMB

#define CFG_TURN_COMB_SIZE 6


turnout_cfg_t * find_last_turn_before(byte subadd);
turnout_cfg_t * find_cfg_turnout(byte subadd);  // find the config in the configs list
turnout_cfg_t * find_cfg_turnout_by_pin(byte pin);  // find the config in the configs list

// EEPROM related functions

bool room_in_eeprom(byte alloc_size);
int ee_find_free_turnout();   // Find the first free slot or -1 if there is no room
int ee_find_cfg_turnout(byte subadd);

turnout_cfg_t * read_cfg_turn(int ee_add);  // read cfg_turnout_t struct in eeprom
void update_cfg_turnout(turnout_cfg_t * cfg,int ee_add=-1); //update turnout cfg in eeprom
void save_cfg_turnout(int ee_add,turnout_cfg_t * cfg);  // Save new cfg turnout at address ee_add

extern int eeprom_turn_end;
extern int eeprom_sensor_end;

extern turnout_cfg_t * turnout_cfg_head;

#endif // TURNOUT_H
