#include "turnouts.h"
#include "sensors.h"
#include "answers.h"
#include "RRduino.h"
#include <EEPROM.h>
#include <Servo.h>

HardwareSerial& to_bus = Serial1;
// Pool of servos top be used
Servo servos[NB_SERVOS];  // servos[0] is reserved for turnout position tuning

#define MAX_COMMAND_LENGTH 50
#define version_count 1

// This pin must be held LOW to enable the "set address mode"
#define ADDRESS_MODE_PIN 2

/*
 * Config commands:
 * 
 * Set the link between a function (turnout, sensor) to pins:
 * <Y S add subadd pin IO [pullup]> : set pin for a sensor: IO=0 => OUTPUT =1=>INPUT
 * <Y T add subadd servo_pin straight_pos throw_pos [straight_pin thrown_pin]>: set servo pin and servo position[and relay pins]
 * Set forbidden combinations for combined turnouts (like 3-way turnout):
 * <Y C add nb_turnouts subadd1 subadd2 [subadd3 ...] 0/1 0/1 ... (number of 0/1 is nb_turnouts) [0/1 ...]>
 * 
 * Turnout command:
 * <T add subadd pos>
 * Immediate answer:
 * <AT[+] OK> or <AT[+] NOK>
 * 
 * Answer (Async):
 * <AT add subadd pos>
 * ...
 * <AE[+]>
 * 
 * Sensor command:
 * <S add subadd> : read sensor
 * <S add subadd value>: write value 
 * 
 * Answer(Immediate):
 * 
 * <AS[+] OK [val]> or <AS[+] NOK>
 * 
 * Communication commands:
 * 
 * Ask to send any pending anwsers (this is to allow async answers)
 * 
 * <A add limit>  (limit=0 => no limit)
 * <AS subadd val>
 * ...
 * <AE[+]>
 * 
 * General commands:
 * 
 * Version:
 * <V>  : Get version, answer <V byte(version)>
 * <V[+] byte(version)> : set version (saved also in eeprom)
 * 
 * Set address:
 * <C address>
 * Answer <AC OK> or <AC NOK>
 * 
 * Setting command:
 * <I address subadd pos> : set turnout subadd to "pos": used to setup the straight and thrown position
 *                          if pos == -1 unset the fine tuning of the turnout (back to normal mode)
 *Answer <AI[+] OK> or <AI[+] NOK>                         
 *
 * Clear EEPROM:
 * <W add>
 * 
 * Show config:
 * <D T add> : show turnouts
 * answer(Immediate):
 * <AT subadd pin straitght_pos thrown_pos [straight_pin thrown_pin]>
 * ...
 * <AT[+]> : end of the turnouts config
 * 
 * <D C add> : show turnouts combination
 * Answer(Immediate):
 * <AC nb_turnouts subadd1... 0/1...>
 * ...
 * <AC[+]>
 * <D S add> : show sensors
 * Answer(Immediate):
 * <AS subadd pin OUTPUT/INPUT/INPUT_PULLUP>
 * ...
 * <AS[+]>
 */
 
byte address;               // This holds the address of this slave

int find_free_servo()  // returns the index of the first free servo, -1 if none
{
  int i = 1;            // Servo 0 is for fine tuning only
  for (;i<NB_SERVOS;i++) {
    if (!servos[i].attached())
      break;
  }
  if (i==NB_SERVOS)
    return -1;
  return i;
}

bool load_turnouts()
{
  address = EEPROM.read(0);
  if (address==255)  // When eeprom has not been set it reads 255
    address = 0;
  if (address==0) {
    eeprom_turn_end = 1;
    return;
  }
  int ee_add = 1;
  byte first = EEPROM.read(1);
  while (first!=0) {
    to_bus.println(first);
    if (first & 0x80) {
      turn_comb_cfg_t * comb = read_cfg_comb(ee_add);
      if (comb) {
        comb->next = turn_comb_cfg_head;
        turn_comb_cfg_head = comb;
      } else return false;
    } else {
      turnout_cfg_t * turn = read_cfg_turn(ee_add);
      if (turn) {
        turn->status= (1<<7) + NO_SERVO;   // sync with eeprom and no servo attached
        turn->next = turnout_cfg_head;
        turnout_cfg_head = turn;
      } else return false;
    }
    ee_add+=CFG_TURN_COMB_SIZE;
    first = EEPROM.read(ee_add);
  }
  eeprom_turn_end = ee_add;
  return true;
}

bool load_sensors()
{
  if (address==0)
    return;
  Serial.println("loading sensors...");    
  eeprom_sensor_end = EEPROM.length()-CFG_SENSOR_SIZE;
  sensors_chng_state=0;
  int ee_add = eeprom_sensor_end;
  byte first = EEPROM.read(ee_add);
  while (first!=0) {
    Serial.println(first, HEX);
    sensor_cfg_t * sensor = read_cfg_sensor(ee_add);
    if (sensor) {
      sensor->next = sensor_cfg_head;
      sensor_cfg_head = sensor;
    } else return false;
    ee_add-=CFG_TURN_COMB_SIZE;
    first = EEPROM.read(ee_add);
    eeprom_sensor_end = ee_add;
  }
  return true;
}

void config_pins()
{
  turnout_cfg_t * turn = turnout_cfg_head;

  while (turn) {
    pinMode(turn->servo_pin, OUTPUT);
    if (turn->relay_pin_1!=0xFE)
      pinMode(turn->relay_pin_1, OUTPUT);
    if (turn->relay_pin_2!=0xFE)
      pinMode(turn->relay_pin_2, OUTPUT);
    turn = turn->next;
  }

  sensor_cfg_t * sensor = sensor_cfg_head;

  while (sensor) {
    if (sensor->status & (1<< SENSOR_BV_IO))
      pinMode(sensor->sensor_pin, (sensor->status & (1 << SENSOR_BV_PULLUP)) ? INPUT_PULLUP : INPUT);
    else
      pinMode(sensor->sensor_pin, OUTPUT);
    sensor = sensor->next;
  }
  // Config "set address mode" pin
  pinMode(ADDRESS_MODE_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Started");
  to_bus.begin(19200);
  to_bus.println("Started also");
  delay(1000);
  load_turnouts();
  load_sensors();
  config_pins();
}

byte cStrPos = 0;
bool command_open = false;
char commandStr[MAX_COMMAND_LENGTH+1];

void turnout_command()
{
  char * end;
  unsigned add = strtoul(commandStr+2, &end,10);


// Check if we are the dest of this message
  if ((end==commandStr+2) || (add!=address)) {
    return;
  }
  Serial.print("Turnout command:");
  char * curr = end;
  unsigned subadd = strtoul(curr, &end,10);
  if (curr==end)
    return;
  curr = end;
  unsigned pos = strtoul(curr, &end,10);
  if (curr==end)
    return;
  turnout_cfg_t * turnout = find_cfg_turnout(subadd);
  if (!turnout)
    return;

  // Make sure position is 0 or 1
  if (pos)
    pos = 1;
  // Put it in "moving state" and set new position
  turnout->status |= (1 << TURNOUT_BV_MOV);
  if (pos)
    turnout->status |= 1 << TURNOUT_BV_POS;
  else
    turnout->status &= ~(1 << TURNOUT_BV_POS);
  Serial.println(turnout->status,HEX);
}

void cfg_sensor()
{
  int add,subadd,pin,status;
  char * current=commandStr+4,*endptr;
  bool ok = true;

  add = strtol(current,&endptr,10); // skip addr
  if (current==endptr)
    ok = false;
  else {
    if (add!=address)
      return; // Not for us
    current = endptr;
    subadd = strtol(current,&endptr,10);
    if ((current==endptr) or (subadd<=0) or (subadd>255)) // check subadd range
      ok = false;
    else {
      current = endptr;
      pin = strtol(current,&endptr,10);
      if ((current==endptr) or (pin<0) or (pin>63)) // check pin range
        ok = false;
      else {
        current = endptr;
        int io=strtol(current,&endptr,10);
        if (current==endptr)
          ok = false;
        else {
          current = endptr;
          if (io) {
            status = 1 << SENSOR_BV_IO;
            int pullup = strtol(current,&endptr,10);
            if (current!=endptr) {
              if (pullup)
                status |= 1 << SENSOR_BV_PULLUP;
            }
          }
        }
      }
    }
  }
  if (!ok) {
    answer_cfg(true,false);
    return;
  }
  sensor_cfg_t * cfg = find_cfg_sensor(subadd);
  if (cfg) {
    // this cfg exists so just update it
    cfg->sensor_pin = pin;
    update_cfg_sensor(subadd,pin,status);
    answer_cfg(false,false);          
    cfg->status|=0x80 | status;
  } else {
    if (!room_in_eeprom(CFG_SENSOR_SIZE)) {
      answer_cfg(true,true);
      return;
    }
    // Allocate new config struct and populate it
    cfg = new sensor_cfg_t;
    if (!cfg) {
      answer_cfg(true,false);
      return;
    }
    // Populate cfg
    cfg->subadd = subadd;
    cfg->sensor_pin = pin;
    cfg->status = 0x80 | status;
    cfg->last_time=millis();
    // add it to the list
    cfg->next = sensor_cfg_head;
    sensor_cfg_head = cfg;  
    // save it to eeprom
    save_cfg_sensor(ee_find_free_sensor(),cfg);
    answer_cfg(false,false);
  }
}

/* Helper, parse add/subadd/pin
 *  Returns NULL if error or the pos of the next field
 */

char * parse_cfg_first_fields(char * cfg_str,int& add,int& subadd,int& pin)
{
  char * current=cfg_str,*endptr;

  add = strtol(cfg_str,&current,10); // skip addr
  if (cfg_str==current)
    return NULL;
  subadd = strtol(current,&endptr,10);
  if ((current==endptr) or (subadd<=0) or (subadd>127)) // check subadd range
    return NULL;
  current = endptr;
  pin = strtol(current,&endptr,10);
  if ((current==endptr) or (pin<0) or (pin>63)) // check pin range
    return NULL;
  return endptr;
}

void cfg_turnout()
{
  int add=0,subadd,pin;
  char * current=parse_cfg_first_fields(commandStr+4,add,subadd,pin);
  if (add!=address)
    return;  // Not for us
  if (!current) {
    answer_cfg(subadd,true);
    return;
  }
  char * endptr = NULL;
  int straight_pos = strtol(current,&endptr,10);
  if ((current==endptr) or (straight_pos<20) or (straight_pos>160)) // check servo pos
  {
    answer_cfg(subadd,true);
    return;
  }
  current = endptr;
  
  int thrown_pos = strtol(current,&endptr,10);
  if ((current==endptr) or (thrown_pos<20) or (thrown_pos>160)) // check servo pos
  {
    answer_cfg(subadd,true);
    return;
  }

  byte relay_pin_1 = 0xFE;  // Special value: means no relay pin
  byte relay_pin_2 = 0xFE;
  endptr++; // skip white space

  if (*endptr)  // If we are not at the end, there are also relay pins
  {
    relay_pin_1 = strtol(current,&endptr,10);
    
    if (current==endptr)
    {
      answer_cfg(subadd,true);
      return;
    }
    
    if (relay_pin_1>=NUM_DIGITAL_PINS)  // check relay pin 1
    {
      answer_cfg(subadd,true);
      return;
    }
    current = endptr;
    relay_pin_2 = strtol(current,&endptr,10);
    if ((current==endptr) || (relay_pin_2>=NUM_DIGITAL_PINS))
    {
      answer_cfg(subadd,true);
      return;
    }
  }
  turnout_cfg_t * cfg = find_cfg_turnout(subadd);
  if (cfg) {
    // exists already, update it
    cfg->servo_pin=pin;
    cfg->straight_pos = straight_pos;
    cfg->thrown_pos = thrown_pos;
    cfg->relay_pin_1 = relay_pin_1;
    cfg->relay_pin_2 = relay_pin_2;
    cfg->next = turnout_cfg_head;
    update_cfg_turnout(cfg);
    answer_cfg(false,false);
  } else {
    // Allocate new config struct and populate it
    int ee_free_slot = ee_find_free_turnout();
    if (ee_free_slot == -1) {
      answer_cfg(true,true);
      return;
    }
    cfg = new turnout_cfg_t;
    if (!cfg) {
      answer_cfg(true,false);
      return;
    }
    cfg->subadd = subadd;
    cfg->servo_pin = pin;
    cfg->straight_pos = straight_pos;
    cfg->thrown_pos = thrown_pos;
    cfg->relay_pin_1 = relay_pin_1;
    cfg->relay_pin_2 = relay_pin_2;
    cfg->next = turnout_cfg_head;
    cfg->status = NO_SERVO;
    turnout_cfg_head = cfg;
    save_cfg_turnout(ee_free_slot,cfg);
    answer_cfg(false,false);
  }
}

void answer_cfg(bool error, bool ee_full)
{
  if ((sensors_chng_state>0) || answers_head)
    to_bus.print("<AY+ ");
  else
    to_bus.print("<AY ");
  if (!error)
    to_bus.print("OK");
  else {
    to_bus.print("NOK");
    if (ee_full)
      to_bus.print(" EE_FULL");
  }
  to_bus.println(">");
}

void cfg_turn_comb()
{
  int add;
  byte subadds[4], forbidden[2], nb_turnouts;
  char * current=commandStr+4,*endptr;
  bool ok = true;

  add = strtol(current,&endptr,10);
  if ((current==endptr) || (add!=address))
    return;
  current = endptr;
  nb_turnouts = strtol(current,&endptr,10);
  if (current==endptr) {
    answer_cfg(true,false);
    return;
  }
  current = endptr;
  int i;
  for (i=0;i<nb_turnouts;i++) {
    subadds[i] = strtoul(current,&endptr,10);
    if (current==endptr) {
      answer_cfg(true,false);
      return;
    }
    current = endptr;
  }
  int j=0;
  do {
    byte forb = strtoul(current,&endptr,2);  // a forbidden combination is just a binary nibble
    if (current==endptr)  {
      if (j>0)
        break; // No more combinations
      // Missing turnout position
      answer_cfg(true,false);
      return;
    }
    
    if  (forb>=(1<<nb_turnouts)) {
      // Bad turnout positions
      answer_cfg(true,false);
      return;
    }
    if (j%2==0)
      forbidden[j/2]=forb;
    else
      forbidden[j/2]|=forb << 4;
    current = endptr;
    j++;
  } while (j<4);
  turn_comb_cfg_t * cfg = find_cfg_turn_comb(subadds);
  if (cfg) {
    // Exists already, update it
    for (i=0;i<2;i++)
      cfg->forbidden[i]=forbidden[i];
    update_cfg_turn_comb(cfg);
    answer_cfg(false,false);
  } else {
    // New combination, allocate it and save it
    // Check room in eeprom
    int ee_add = ee_find_free_turnout();
    if (ee_add == -1) {
      answer_cfg(true,true);
      return;
    }
    cfg = new turn_comb_cfg_t;
    if (!cfg) {
      answer_cfg(true, false);
      return;
    }
    int i=0;
    for (;i<4;i++)
      cfg->subadds[i] = subadds[i];
    for (i=0;i<2;i++)
      cfg->forbidden[i]=forbidden[i];
    save_cfg_turn_comb(ee_add,cfg);
    answer_cfg(false,false);   
  }
}

void cfg_command()
{
  char * end;
  unsigned add = strtoul(commandStr+4, &end,10);

  // Check if we are the dest of this message
  if ((end==commandStr+4) || (add!=address)) {
    return;
  }
  switch (commandStr[2]) {
  case 'S':
    cfg_sensor();
    break;
  case 'T':
    cfg_turnout();
    break;
  case 'C':
    cfg_turn_comb();    
  }   
}

void sensor_answer(bool error)
{
  if (error) {
    // FIXME: better error indication
    if ((sensors_chng_state>0) || answers_head)
      to_bus.println("<AS+ NOK>");
    else
      to_bus.println("<AS NOK>");
  }
  else {
    if ((sensors_chng_state>0) || answers_head)
      to_bus.println("<AS+ OK>");
    else
      to_bus.println("<AS OK>");
  }
}
void sensor_command()
{
  char * end;
  unsigned add = strtoul(commandStr+2, &end,10);

// Check if we are the dest of this message
  if ((end==commandStr+2) || (add!=address)) {
    return;
  }
  char * curr = end;
  unsigned subadd = strtoul(curr, &end,10);
  bool error = true;
  sensor_cfg_t * sensor;
  if (curr!=end) {
    sensor = find_cfg_sensor(subadd);
    if (sensor) error = false;
  }
  if (error) {
    sensor_answer(true);
    return;
  }
  if (sensor->status & (1 << SENSOR_BV_IO)) {
    // Input sensor, send last validated state
    byte val = sensor->status & 0x1;
    if (sensor->status & (1 << SENSOR_BV_CHNG_STATE))
    {
      Serial.println(sensor->status,HEX);
      sensor->status &= ~(1 << SENSOR_BV_CHNG_STATE);
      Serial.println(sensor->status,HEX);
      
      if (sensors_chng_state>0)
        sensors_chng_state--;
      else Serial.println("Sensors------!!");
    }
    char answer[16];
    if ((sensors_chng_state>0) || answers_head)
      snprintf(answer,16,"<AS+ OK %d>", val?1:0);
    else
      snprintf(answer,16,"<AS OK %d>", val?1:0);
    to_bus.println(answer);
  } else {
    curr = end;
    unsigned pos = strtoul(curr, &end,10);
    if (curr==end) {
      sensor_answer(true);
      return;
    }
    digitalWrite(sensor->sensor_pin,pos ? HIGH: LOW);
    Serial.print(sensor->sensor_pin);
    Serial.print(" ");
    Serial.println(pos);
  }  
}

bool in_range(byte val,byte bound_1,byte bound_2)
{
  byte m = min(bound_1,bound_2);
  byte M = max(bound_1,bound_2);

  return (val>=m) && (val<=M);
}

// Move all turnouts that have been activated
void process_turnouts()
{
  turnout_cfg_t * cur = turnout_cfg_head;
  while (cur) {
    if (cur->status & (1 << TURNOUT_BV_MOV)) {
      if ((cur->status & ~0x1F)==0)  // turnout is being fine tuned, dont move it
        return;
      // check if this is a new movment, in that case, attach the servo
      if ((cur->status & 0x01F)==NO_SERVO) {
        int servo_index = find_free_servo();
        if (servo_index ==-1)
          // No free servo just return, we'll try later
          return;
        cur->status = (cur->status & 0xE0) + (servo_index & 0x1F); // set the servo index
        // Put the correct current position
        if (cur->status & (1 << TURNOUT_BV_POS))
          cur->current_pos = cur->straight_pos;
        else
          cur->current_pos = cur->thrown_pos;
        servos[cur->status & 0x01F].write(cur->current_pos);  
        servos[cur->status & 0x01F].attach(cur->servo_pin);
      }
      else if (!in_range(cur->current_pos,cur->straight_pos,cur->thrown_pos)) {
        // Movement is done, detach the servo
        servos[cur->status & 0x1F].detach();
        cur->status = (cur->status &0xE0 & ~(1 << TURNOUT_BV_MOV)) + NO_SERVO;  // unset servo index and movement bit
        Serial.print(cur->status);
        // Send answer to master node
        char answer[20];
        snprintf(answer,20,"<AT %d %d %d>",address,cur->subadd,(cur->status >> TURNOUT_BV_POS)&0x01);
        Serial.println(answer);
        queue_answer(answer);
      }
      // FIXME: a verifier et ajouter les relais
      else servos[cur->status & 0x1F].write(cur->current_pos);
      // dir is -1 if going from straight to thrown pos is done by incrementing, -1 otherwise
      int dir = (cur->thrown_pos>cur->straight_pos) ? 1 : -1;
      Serial.println(cur->current_pos);
      if (cur->status & (1 << TURNOUT_BV_POS)) // moving from straight to thrown, use dir to increment
        cur->current_pos += dir;
      else
        cur->current_pos -= dir;
    }
    cur=cur->next;
  }
}

void version_command()
{
  char * end;
  unsigned add = strtoul(commandStr+2, &end,10);
  // Check if we are the dest of this message
  if ((end==commandStr+2) || (add!=address)) {
    return;
  }
  char answer[20];
  snprintf(answer,20,"<AV %d %d>",address,version_count);
  EEPROM.write(0,add);
  to_bus.println(answer);
}

void set_address()
{
  char * end;
  int add=-1;
  if (commandStr[1]!=0)
    add = strtoul(commandStr+2, &end,10);
  if ((end!=commandStr+2) && (add>0) && (add<255)) // Sanity checks
  {
    digitalWrite(13,LOW);
    address = add;
    EEPROM.write(0,add);
    to_bus.println("<AC OK>");
  }
  else to_bus.println("<AC NOK>");
}

void set_turnout_pos()
{
  char * end;
  unsigned add = strtoul(commandStr+2, &end,10);

// Check if we are the dest of this message
  if ((end==commandStr+2) || (add!=address)) {
    return;
  }
  char * curr = end;
  unsigned subadd = strtoul(curr, &end,10);
  if (curr==end) {
    return;
  }
  curr = end;
  int pos = strtol(curr,&end,10);
  if (curr==end) {
    return;
  }
  turnout_cfg_t * turn = find_cfg_turnout(subadd);
  if (turn) {
    if ((sensors_chng_state>0) || answers_head)
      to_bus.println("<AI+ OK>");
    else
      to_bus.println("<AI OK>");
    // Set the turnout servo to 0, which is a special servo dedicated to turnout tuning
    // This will also prevent the code to throw or unthrow the turnout while its position is being set
    if ((pos>=10) && (pos<=170)) {
      if ((turn->status & 0x1F!=0) &&(turn->status & 0x1F!=NO_SERVO)) // Check if it was attached to a servo
        servos[turn->status & 0x1F].detach();
      turn->status &= 0xE0;   // Means: reserved for fine tuning
      servos[0].detach();
      servos[0].attach(turn->servo_pin);
      servos[0].write(pos);     // Set position
    }
    else if (pos==-1)
      turn->status = (turn->status & 0xE0)+NO_SERVO;  // Back to normal
  }
}

void wipe_out()
{
  char * end;
  unsigned add = strtoul(commandStr+2, &end,10);

// Check if we are the dest of this message
  if ((end==commandStr+2) || (add!=address)) {
    return;
  }
  Serial.println("wiped out");
  if ((sensors_chng_state>0) || answers_head)
    to_bus.println("<AW+ OK>");
  else
    to_bus.println("<AW OK>");
  EEPROM.write(0,0);  // Address is set to 0, this will prevent the whole EEPROM saving
  address = 0;
  // Wipe turnouts and sensors out
  EEPROM.write(EE_BEGIN_TURN,0);
  EEPROM.write(EEPROM.length()-CFG_SENSOR_SIZE,0);
}

void answers_command()
{
  char * end;
  unsigned add = strtoul(commandStr+2, &end,10);

  // Check if we are the dest of this message
  if ((end==commandStr+2) || (add!=address)) {
    return;
  }
  char * curr = end;
  unsigned limit = strtoul(curr, &end,10);
  if (curr==end)
    return;
  send_pending_answers(limit);
}

void display_end(char type)
{
  to_bus.print("<A");
  to_bus.print(type);
  if ((sensors_chng_state>0) || answers_head)
    to_bus.print("+");
  to_bus.println(">");
}
void display_command()
{
  char * end;
  int add = strtoul(commandStr+4,&end,10);
  if ((end !=commandStr+4) && (add==address)) {
    switch (commandStr[2]) {
    case 'S': {
        char str[18];
        if (str==NULL)
          Serial.println("STR NULL");
        for (sensor_cfg_t * sens=sensor_cfg_head;sens;sens=sens->next) {
          Serial.println("sensor cfg to str");
          sensor_cfg_to_str(sens,str);
          to_bus.println(str);
        }
      }
      break;
    case 'T': {
        char str[32];
        for (turnout_cfg_t * cfg=turnout_cfg_head;cfg;cfg=cfg->next) {
          turnout_cfg_to_str(cfg,str);
          to_bus.println(str);
        }
      }
      break;
    case 'C': {
        char str[45];
        for (turn_comb_cfg_t * cfg=turn_comb_cfg_head;cfg;cfg=cfg->next) {
          turn_comb_cfg_to_str(cfg,str);
          to_bus.println(str);
        }
        to_bus.print("<AC");
        if ((sensors_chng_state>0) || answers_head)
          to_bus.print("+");
        to_bus.println(">");
      }
      break;
    default:
      to_bus.println("unknown command");
      return;
    }
    display_end(commandStr[2]);
  }
}

long last_run = 0;
bool blinking=false;

void loop() {
  if (to_bus.available()) {
    char c = to_bus.read();
    Serial.write(c);
    if (c=='<') {
      // a new command begins
      cStrPos = 0;
      command_open = true;
    } else if (command_open) {
      if (c=='>') {
        // Command is complete
        commandStr[cStrPos]='\0';
        Serial.println(commandStr);
        Serial.println(address);
        switch (commandStr[0]) {
          case 'T':
            if (address!=0)
              turnout_command();
            break;
          case 'Y':
            if (address!=0)
              cfg_command();
            break;
          case 'S':
            if (address!=0)
              sensor_command();
            break;
          case 'V':
            if (address!=0)
              version_command();
            break;
          case 'C':
            if (digitalRead(ADDRESS_MODE_PIN)==LOW)
              set_address();
            break;
          case 'I':
            if (address!=0)
              set_turnout_pos();
            break;
          case 'A':
            if (address!=0)
              answers_command();
            break;
          case 'D':
            if (address!=0)
              display_command();
            break;
          case 'W':
            if (address!=0)
              wipe_out();
        }
        command_open = false;
      }
      else if (cStrPos < MAX_COMMAND_LENGTH)
        commandStr[cStrPos++]=c;
        // We discard all characters when the buffer is full
    }
  }
  if (address == 0) {
    // address == 0 => we wait for a special command from the master node
    // <C add> : sets the address of the slave to add: only one must be listening though
    if (millis()-last_run>500) {
      last_run = millis();
      blinking = ! blinking;
      if (digitalRead(ADDRESS_MODE_PIN)==LOW)
        digitalWrite(13, blinking?HIGH:LOW);
      else digitalWrite(13,HIGH);
    }
  }
  // Process moving turnouts every 10ms
  else if ((millis()/10)!=last_run) {
    process_turnouts();
    last_run = millis()/10;
  }
  check_all_sensors();
}
