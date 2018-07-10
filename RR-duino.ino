#include "turnouts.h"
#include "sensors.h"
#include "answers.h"
#include "RRduino.h"
#include <EEPROM.h>
#include <Servo.h>

HardwareSerial& to_bus = Serial1;
// Pool of servos top be used
Servo servos[NB_SERVOS];  // servos[0] is reserved for turnout position tuning
unsigned long relay_time[NB_SERVOS];

byte version_nb;
bool address_mode = false;

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
  if (address==255) { // When eeprom has not been set it reads 255
    address = 0;
    version_nb = 0;
  } else version_nb = EEPROM.read(1);
  if (address==0) {
    eeprom_turn_end = 2;
    return;
  }
  int ee_add = 2;
  byte first = EEPROM.read(2);
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
        // Add turnout so that the list of turnouts is sorted in ascending order with respect to subaddress
        turnout_cfg_t * place = find_last_turn_before(turn->subadd);
        if (place) {
          turn->next = place->next;
          place->next = turn;
        } else {
          turn->next = turnout_cfg_head;
          turnout_cfg_head = turn;
        }
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
      // Add sensor so that the list of sensors is sorted in ascending order with respect to subaddress
      sensor_cfg_t * place = find_last_sensor_before(sensor->subadd);
      if (place) {
        sensor->next = place->next;
        place->next = sensor;
      } else {
        sensor->next = sensor_cfg_head;
        sensor_cfg_head = sensor;
      }
    } else return false;
    ee_add-=CFG_TURN_COMB_SIZE;
    first = EEPROM.read(ee_add);
    eeprom_sensor_end = ee_add;
  }
  return true;
}

// Config pin I/O and also set the outputs to their last known state
void config_pins()
{
  turnout_cfg_t * turn = turnout_cfg_head;

  while (turn) {
    pinMode(turn->servo_pin, OUTPUT);
    // Set up relay pin, 0xFE means not used
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
    else {
      pinMode(sensor->sensor_pin, OUTPUT);
      if (sensor->status & 0x01)
        digitalWrite(sensor->sensor_pin, HIGH);
      else
        digitalWrite(sensor->sensor_pin, LOW);
    }
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
  for (byte i=0;i<NB_SERVOS;i++)
    relay_time[i]=0;
}

byte cmd_pos = 0;
bool new_command = false;
byte command_buf[MAX_CMD_LEN];

void send_simple_answer(byte err) {
  to_bus.write(0xFF);
  command_buf[0]&=~(1<<CMD_CMD_ANSWER_BV); // Unset command bit
  command_buf[2]=0x80 | err;  // add error code
  to_bus.write(0xFF);
  for (byte i;i<3;i++)
    to_bus.write(command_buf[i]);
}

int write_one_turnout(byte subadd)
{
  Serial.print("Turnout command:");
  Serial.println(subadd);
  turnout_cfg_t * turnout = find_cfg_turnout(subadd & 0x3F);
  if (!turnout) {
    Serial.println("Unknown turnout!");
    return -UNKNOWN_DEV;
  }
  byte pos = (subadd >> SUB_VALUE_BV) & 0x01;
  // Put it in "moving state" and set new position
  turnout->status |= (1 << TURNOUT_BV_MOV);
  if (pos)
    turnout->status |= 1 << TURNOUT_BV_POS;
  else
    turnout->status &= ~(1 << TURNOUT_BV_POS);
  Serial.println(turnout->status,HEX);
  return 0; // No error
}

int write_one_sensor(byte subadd)
{
  sensor_cfg_t * sensor;
  Serial.print("write sensor command:");
  Serial.println(subadd);
  sensor = find_cfg_sensor(subadd & 0x3F);
  if (!sensor) {
    Serial.println("unknown sensor");
    return -UNKNOWN_DEV;
  }
  if (sensor->status & (1 << SENSOR_BV_IO)) {
    Serial.println("Trying to write on input sensor");
    return -UNVALID_DEV;
  }
  byte pos = (subadd >> SUB_VALUE_BV) & 0x01;
  if (pos) {
    digitalWrite(sensor->sensor_pin, HIGH);
    sensor->status |= 0x01;
  } else {
    digitalWrite(sensor->sensor_pin, LOW);
    sensor->status &= 0xFE;
  }
  sensor->status &= ~(1 << SENSOR_BV_SYNC); // No more synced in EEPROM
  Serial.print(sensor->sensor_pin);
  Serial.print(" ");
  Serial.println(pos);
  return 0;  // No error
}

// Write to several turnouts/sensors
void write_several()
{
  int (*func)(byte); // pointer on the func to be called
  // choose turnout/sensor according to command byte
  if (command_buf[0] & CMD_SENS_TURN_BV)
    func = &write_one_turnout;
  else
    func = &write_one_sensor;
    
  for (byte pos = 2;command_buf[pos]!=0x80;pos++) {
    int err = (*func)(command_buf[pos]);
    if (err<0)
    {
      command_buf[0]&=~(1<<CMD_CMD_ANSWER_BV); //Unset command bit
      command_buf[pos]=0x80 | (-err);// set error code
      send_one_msg(command_buf,pos+1);
      return;
    }
  }
  send_simple_answer(0); // No error
}

void write_all_turnouts()
{
  Serial.println("Writing to all turnouts");
  send_simple_answer(0);
}

void write_all_sensors()
{
  Serial.println("Writing to all sensors");
  send_simple_answer(0);
}

// read one sensor and returns its value
// also unset the changed state if it was set and correct the sensors changed state counter
int read_one_sensor(byte subadd)
{
  sensor_cfg_t * sensor;
  Serial.print("read sensor command:");
  Serial.println(subadd);
  sensor = find_cfg_sensor(subadd & 0x3F);
  if (!sensor) {
    Serial.println("unknown sensor");
    return -UNKNOWN_DEV;
  }
  byte val = sensor->status & 0x1;
  if (sensor->status & (1 << SENSOR_BV_IO)) {
    // Input sensor, send last validated state

    if (sensor->status & (1 << SENSOR_BV_CHNG_STATE))
    {
      Serial.println(sensor->status,HEX);
      sensor->status &= ~(1 << SENSOR_BV_CHNG_STATE);
      Serial.println(sensor->status,HEX);
      
      if (sensors_chng_state>0)
        sensors_chng_state--;
      else Serial.println("Read one sensor that has changed state set, unset it");
    }
  }
  return val;
}

// read one turnout and returns its value
int read_one_turnout(byte subadd)
{
  turnout_cfg_t * turn;
  Serial.print("read turnout command:");
  Serial.println(subadd);
  turn = find_cfg_turnout(subadd & 0x3F);
  if (!turn) {
    Serial.println("unknown turnout");
    return -UNKNOWN_DEV;
  }
  byte val = (turn->status >> TURNOUT_BV_POS) & 0x01;
  return val;
}

void read_several()
{
  int (*func)(byte);
  if (command_buf[0] & CMD_SENS_TURN_BV)
    func = &read_one_turnout;
  else
    func = &read_one_sensor;
  // Modify command_buf to become the answrt
  command_buf[0] &= ~(1<<CMD_CMD_ANSWER_BV);  // Unset command bit, it is an answer
  for (byte pos=2;pos<cmd_pos-1;pos++)  // go over all subadresses, do not change the last 0x80
  {
    int val = (*func)(command_buf[pos] & 0x3F);
    if (val<0) {
      command_buf[pos]=0x80 | (-val); // set error code
      send_one_msg(command_buf,pos);
      return;
    }
    if (val)
      command_buf[pos] |= (1<< SUB_VALUE_BV);
    else
      command_buf[pos] &= ~(1<< SUB_VALUE_BV);
  }
  send_one_msg(command_buf, cmd_pos);  // no error occured so we do not change the last 0x80 from the command
}

void read_all_sensors()
{
  Serial.println("reading all sensors");
  send_simple_answer(0);
}

void read_all_turnouts()
{
  Serial.println("reading all turnouts");
  send_simple_answer(0);
}

// data points to the part of a buffer where the function puts
// the subaddress and the pin number with correct bits set (I/O, pullup,...)
// returns the number of bytes used (always 2 for sensors)
byte show_one_sensor(sensor_cfg_t * sensor, byte * data)
{
  data[0]=sensor->subadd;  
  data[1]=sensor->sensor_pin;
  // Set I/O bit
  if (!(sensor->status & (1 << SENSOR_BV_IO)))
    data[0] |= (1<<SUB_IODIR_BV); // output
  else { // input
    if (sensor->status & (1 << SENSOR_BV_PULLUP)) // set pullup
      data[1] |= (1 << PIN_PULLUP_BV);
  }
  return 2;
}

// data points to the part of a buffer where the function puts
// the subaddress and the pin number with correct bits set (I/O, pullup,...)
// returns the number of bytes used
byte show_one_turnout(turnout_cfg_t * turn, byte * data)
{
  data[0]=turn->subadd;  
  data[1]=turn->servo_pin;
  data[2]=turn->straight_pos;
  data[3]=turn->thrown_pos;
  if ((turn->relay_pin_1!=254) || (turn->relay_pin_2!=254))
  {
    // Set relay_pin bit
    data[0] |= (1<<SUB_RELAY_PIN_BV);
    data[4]=turn->relay_pin_1 | (1<<PIN_RELAY_PULSE_BV); // For now only latching relays so pulse
    data[5]=turn->relay_pin_2 | (1<<PIN_RELAY_PULSE_BV); // For now only latching relays so pulse
    return 6;
  }
  return 4;
}

void show_sensors_cmd(byte limit)
{
  if (!answers_head)  // if no answer is waiting this means it is the first show command
  { // build the answers
    byte pos = 2; // beginning of the payload of the command
    command_buf[0] &= ~(1 << CMD_CMD_ANSWER_BV); // Unset command bit ( this is an answer )
    for (sensor_cfg_t * sensor = sensor_cfg_head;sensor;sensor=sensor->next) {
      pos += show_one_sensor(sensor,command_buf+pos);
      if (pos+3>MAX_CMD_LEN) // not enough place for another sensor config so queue the current answer
      {
        byte * data = (byte*) new byte[pos+1]; // Allocate mem
        memcpy(data, command_buf, pos+1); // copy answer
        data[pos]=0x80; // Add termination
        queue_answer(answers_head, data, pos+1);
        pos = 2; // reset position
      }
    }
  }
  send_answers(limit); // send them; some might not be sent (depending on the limit), they will be sent on the next show command
}

void show_turnouts_cmd(byte limit)
{
  if (!answers_head)  // if no answer is waiting this means it is the first show command
  { // build the answers
    byte pos = 2; // beginning of the payload of the command
    command_buf[0] &= ~(1 << CMD_CMD_ANSWER_BV); // Unset command bit ( this is an answer )
    for (turnout_cfg_t * turn = turnout_cfg_head;turn;turn=turn->next) {
      pos += show_one_turnout(turn,command_buf+pos);
      if (pos+7>MAX_CMD_LEN) // not enough place for another turnout config so queue the current answer
      {
        byte * data = (byte*) new byte[pos+1]; // Allocate mem
        memcpy(data, command_buf, pos+1); // copy answer
        data[pos]=0x80; // Add termination no error
        queue_answer(answers_head, data, pos+1);
        pos = 2; // reset position
      }
    }
  }
  send_answers(limit); // send them; some might not be sent (depending on the limit), they will be sent on the next show command
}

// Read the config of a turnout from the command buf at pos
// update or create a new turnout
// returns the number of bytes read from the command buf, or a negative number in case of error
int config_one_turnout(byte pos)
{
  byte subadd = command_buf[pos] & 0x3F;
  byte cfg_size = 4;
  
  turnout_cfg_t * cfg = find_cfg_turnout(subadd & 0x3F);
  if (cfg) {
    // exists already, update it
    cfg->servo_pin=command_buf[pos+1] & 0x7F;  // For now all relays are pulsed (latching relays)
    cfg->straight_pos = command_buf[pos+2];
    cfg->thrown_pos = command_buf[pos+3];
    if (command_buf[pos] & (1<<SUB_RELAY_PIN_BV)) { // relay pins present
      cfg->relay_pin_1 = command_buf[pos+4];
      cfg->relay_pin_2 = command_buf[pos+5];
      cfg_size = 6;
    } else { // no relay pins
      cfg->relay_pin_1 = 0xFE;
      cfg->relay_pin_2 = 0xFE;
    }
    update_cfg_turnout(cfg);
  } else {
    // Allocate new config struct and populate it
    int ee_free_slot = ee_find_free_turnout();
    if (ee_free_slot == -1) {
      Serial.println("EEPROM full");
      return -EEPROM_FULL;
    }
    cfg = new turnout_cfg_t;
    if (!cfg) {
      Serial.println("Memory full");
      return -MEMORY_FULL;
    }
    cfg->subadd = subadd;
    cfg->servo_pin = command_buf[pos+1] & 0x7F;  // For now all relays are pulsed (latching relays)
    cfg->straight_pos = command_buf[pos+2];
    cfg->thrown_pos = command_buf[pos+3];
    if (command_buf[pos] & (1<<SUB_RELAY_PIN_BV)) { // relay pins present
      cfg->relay_pin_1 = command_buf[pos+4];
      cfg->relay_pin_2 = command_buf[pos+5];
      cfg_size = 6;
    } else { // no relay pins
      cfg->relay_pin_1 = 0xFE;
      cfg->relay_pin_2 = 0xFE;
    }
    cfg->status = NO_SERVO;
    // Put the config into the list, but respect the order by the subaddress
    turnout_cfg_t * place = find_last_turn_before(cfg->subadd);
    if (place) {
      cfg->next = place->next;
      place->next = cfg;
    } else {
      cfg->next = turnout_cfg_head;
      turnout_cfg_head = cfg;
    }

    save_cfg_turnout(ee_free_slot,cfg);
  }
  return cfg_size;
}

// Read the config of a sensor from the command buf at pos
// update or create a new sensor
// returns the number of bytes read from the command buf, or a negative number in case of error
int config_one_sensor(byte pos)
{
  sensor_cfg_t * cfg = find_cfg_sensor(command_buf[pos] & 0x3F);
  byte status = 0;
  // Set bits for status
  if (!(command_buf[pos] & (1<<SUB_IODIR_BV)))
    status |= (1<<SENSOR_BV_IO);
  if (command_buf[pos+1] & (1<<PIN_PULLUP_BV)) // Check pullup
    status |= (1 << SENSOR_BV_PULLUP);
  if (cfg) {
    // this cfg exists so just update it
    cfg->sensor_pin = command_buf[pos+1] & 0x7F;
    update_cfg_sensor(command_buf[pos]&0x3F,cfg->sensor_pin,status);       
    cfg->status=status | (1<<SENSOR_BV_SYNC);
  } else {
    if (!room_in_eeprom(CFG_SENSOR_SIZE)) {
      Serial.println("EEPROM full");
      return -EEPROM_FULL;
    }
    // Allocate new config struct and populate it
    cfg = new sensor_cfg_t;
    if (!cfg) {
      Serial.println("Memory full");
      return -MEMORY_FULL;
    }
    // Populate cfg
    cfg->subadd = command_buf[pos] & 0x3F;
    cfg->sensor_pin = command_buf[pos+1]& 0x7F;
    cfg->status = status;
    cfg->last_time=millis();
    // add it to the list
    cfg->next = sensor_cfg_head;
    sensor_cfg_head = cfg;  
    // save it to eeprom
    save_cfg_sensor(ee_find_free_sensor(),cfg);
    cfg->status |= (1<<SENSOR_BV_SYNC);
  }
  return 2;
}

void config_several()
{
  int (*func)(byte);
  if (command_buf[0] & CMD_CFG_SENS_TURN_BV)
    func = &config_one_turnout;
  else
    func = &config_one_sensor;
  // Modify command_buf to become the answer
  for (byte pos=2;pos<cmd_pos;)  // go over all subadresses
  {
    int val = (*func)(pos);
    if (val<0) { // error
      send_simple_answer(-val);
      return;
    }
    pos += val; // advance to next config
  }
  send_simple_answer(0);  // no error occured
}

/*
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
*/
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
        // Queue async event
        queue_async_turnout(cur);
      }
      else {
        servos[cur->status & 0x1F].write(cur->current_pos);
        if (relay_time[cur->status & 0x1F]!=0) { // relay pulse already began
          if (millis()>relay_time[cur->status & 0x1F]+RELAY_PULSE_LEN) { // pulse must be ended
            if (cur->status & (1 << TURNOUT_BV_POS))  // which position
              digitalWrite(cur->relay_pin_2, LOW);
            else
              digitalWrite(cur->relay_pin_1, LOW);
            relay_time[cur->status & 0x1F]=0;
          }
        } 
        else if (cur->current_pos == abs(cur->straight_pos-cur->thrown_pos)/2)
        {
          if ((cur->status & (1 << TURNOUT_BV_POS)) && (cur->relay_pin_2 != 0xFE)) { // which position
              digitalWrite(cur->relay_pin_2, HIGH);  // begin pulse
              relay_time[cur->status & 0x1F]=millis();
          }
          else if (!(cur->status & (1 << TURNOUT_BV_POS)) && (cur->relay_pin_1 != 0xFE)) {
              digitalWrite(cur->relay_pin_1, HIGH);
              relay_time[cur->status & 0x1F]=millis();
          }
        }
      }
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

void version_cmd()
{
  byte msg[]={1<<CMD_CFGCMD_BV,address,version_nb};
  send_one_msg(msg,3);
}

void set_address(byte add)
{
  if (digitalRead(ADDRESS_MODE_PIN)==LOW) {
    digitalWrite(13,LOW);
    address = add;
    EEPROM.write(0,add);
    send_simple_answer(0);
  }
}
/*
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
*/
void clear_eeprom()
{
  Serial.println("wiped out");
  EEPROM.write(0,0);  // Address is set to 0, this will prevent the whole EEPROM saving
  address = 0;
  // Wipe turnouts and sensors out
  EEPROM.write(EE_BEGIN_TURN,0);
  EEPROM.write(EEPROM.length()-CFG_SENSOR_SIZE,0);
  send_simple_answer(0);
}

void async_cmd(byte limit)
{
  Serial.println("Sending async events");
  send_async_events(limit);
}

// send msg on the bus
void send_one_msg(byte * msg,byte len)
{
  if ((sensors_chng_state>0) || async_head)
    msg[0] |= (1 << CMD_PEND_ANSWERS_BV) | (1<<CMD_ASYNC_BV);  // Set pending async events
  to_bus.write(0xFF); // Start byte
  for (byte i=0;i<len;i++)
    to_bus.write(msg[i]);
}

long unsigned last_run = 0;
bool (*check_cmd)(void)=NULL;  // function pointer to the current check command function

// Check and decode show command
bool check_show_cmd_2nd_stage()
{
  if (cmd_pos==3) { // complete!
    if (command_buf[0] & (1 << CMD_CFG_SENS_TURN_BV))
      show_turnouts_cmd(command_buf[2]);
    else
      show_sensors_cmd(command_buf[2]);
    return true;
  }
  return false;
}

bool check_turnout_fine_cmd_2nd_stage()
{
  
}

// returns true if pos is the beginning of a turnout config
bool beg_turnout_cfg(byte pos)
{
  byte cur = 2;
  while (cur<pos) {
    if (command_buf[cur+1] & (1<<SUB_RELAY_PIN_BV))
      cur+=6; // Add 2 for the relay pins
    else cur+=4; // subadd + servo pin + 2 turnout positions
  }
  return cur==pos;
}

bool check_cfgcmd_2nd_stage()
{
  if (cmd_pos < 4) // We need at least one config which is 2 bytes min +command+address bytes
    return false;
  if (command_buf[1] & (1<<ADD_LIST_BV)) { //Several configs
    if (command_buf[0] & (1<<CMD_CFG_SENS_TURN_BV)) { // turnout config
      //check if we have just begun a turnout config and if it is a termination (0x80)
      if (beg_turnout_cfg(cmd_pos-1) && (command_buf[cmd_pos-1]=0x80)) { //complete!
        config_several();
        return true;
      }
    } else { // sensor config
      //check if we have just begun a sensor config and if it is a termination (0x80)
      if (((cmd_pos-1)%2==0) && (command_buf[cmd_pos-1]=0x80)) { //complete!
        config_several();
        return true;
      }      
    }
    return false;
  }
  // Only one config
  if (command_buf[0] & (1<<CMD_CFG_SENS_TURN_BV)) { // turnout config
    byte cfg_size = 2 + 4;
    if (command_buf[1] & (1<<SUB_RELAY_PIN_BV))
      cfg_size+=2;
    if (cmd_pos == cfg_size) { // complete!
      int err = config_one_turnout(2);
      if (err<0)
        send_simple_answer(-err);
      else
        send_simple_answer(0);
      return true;
    }
  } else { // sensor config
    if (cmd_pos == 2+2) { // complete!
      int err = config_one_sensor(2);
      if (err<0)
        send_simple_answer(-err);
      else
        send_simple_answer(0);
      return true;
    }
  }
  return false;
}

bool check_rwcmd_2nd_stage()
{
  if (cmd_pos<2)
    return false;
  if (command_buf[0] & (1 << CMD_ALL_BV)) { // read or write ALL
    if (!(command_buf[0] & (1 << CMD_RWDIR_BV))) // read all command
    {
      if (command_buf[0] & (1 << CMD_SENS_TURN_BV))
        read_all_turnouts();
      else
        read_all_sensors();
      return true;
    }
    // Write all command: check if last byte has MSB set (termination)
    if ((cmd_pos>=3) && (command_buf[cmd_pos-1] & (1 << 7))) { // Complete!
      if (command_buf[0] & (1 << CMD_SENS_TURN_BV))
        write_all_turnouts();
      else
        write_all_sensors();
      return true;
    }
    return false;
  }
  if (command_buf[1] & (1 << ADD_LIST_BV)) { // List of subaddresses, terminated by 0x80
    if ((cmd_pos>=3) && (command_buf[cmd_pos-1]==0x80)) // Complete!
    {
      if (command_buf[0] & (1 << CMD_RWDIR_BV)) // Write several subaddresses
        write_several();
      else
        read_several();
      return true;
    }
    return false;
  }
  // Read or write on one subaddress only
  if (cmd_pos == 3) { // Complete!
    if (command_buf[0] & (1 << CMD_RWDIR_BV)) {// Write
      if (command_buf[0] & (1 << CMD_SENS_TURN_BV))
        write_one_turnout(command_buf[2]);
      else
        write_one_sensor(command_buf[2]);
    }
    else { //Read
      int err;
      if (command_buf[0] & (1 << CMD_SENS_TURN_BV))
        err = read_one_turnout(command_buf[2]);
      else
        err = read_one_sensor(command_buf[2]);
      if (err<0)
        send_simple_answer(-err);
      else {
        command_buf[0] &= ~(1<<CMD_CMD_ANSWER_BV); //Unset command bit
        command_buf[2] |= (err << SUB_VALUE_BV);
        send_one_msg(command_buf,3);
      }
    }
    return true;
  }
  return false;
}

// Checks if address is correct and command is complete then calls the corresponding function
// for simple commands, otherwise set the check command pointer to the next stage check function
// if command has been processed or address is not ours returns true, false otherwise
bool check_cmd_1st_stage()
{
  if (!(command_buf[0] & CMD_CMD_ANSWER_BV))  // If this is an answer, it is not for us
    return true;
  if ((!address_mode) && ((cmd_pos==2) && (command_buf[1]!=address))) // check address unless we are in address mode
    return true;

  if (command_buf[0] & (1 << CMD_ASYNC_BV)) {
    // ASYNC command
    if (cmd_pos == 2) // complete!
    {
      async_cmd(command_buf[0] >> 3);
      return true;  
    }
    return false;
  }
  if (command_buf[0] & (1 << CMD_CFGCMD_BV))  // Config commands
  {
    if (command_buf[0] & (1 << CMD_CFG_SPECIAL_BV))  //Special config commands
    {
      switch ((command_buf[0] >> 5)&0x03) {
        case 0b00: //Version command
          if (cmd_pos==2) // complete!
          {
            version_cmd();
            return true;
          }
          return false;
        case 0b01:  // Set address command
          if (cmd_pos==2) // complete!
          {
            set_address(command_buf[1]);
            return true;
          }
          return false;
        case 0b10:
          // show command
          check_cmd = &check_show_cmd_2nd_stage;
          return false;
        case 0b11:
          if (command_buf[1] & (1 << CMD_CFG_SPECIAL_EEPROM_BV)) {  // Clear EEPROM
            clear_eeprom();
            return true;
          }
          // Turnout fine tuning, set the check function accordingly
          check_cmd = & check_turnout_fine_cmd_2nd_stage;
          return false;
      }
    }
    check_cmd = & check_cfgcmd_2nd_stage;  // it is a normal config command set the check cmd accordingly
    return false;
  }
  check_cmd = &check_rwcmd_2nd_stage;  //now set the check cmd pointer to the 2nd stage rw command
  return false;
}

byte blinking = HIGH;
unsigned long last_blink=0;
void loop() {
  if ((millis()/10)!=last_run) {  // move turnouts and check sensors every 10 ms
    // Check address mode pin, solid led to indicate we are waiting for an address
    if (digitalRead(ADDRESS_MODE_PIN)==LOW) {
      digitalWrite(13,HIGH);
      address_mode = true;
    } else address_mode = false;

    process_turnouts();
    last_run = millis()/10;
    check_all_sensors();
  }
  // Command processing
  if (to_bus.available()) {
    byte c = to_bus.read();
    if (c == START_BYTE) {
      cmd_pos = 0;  // new command
      new_command = true;
      check_cmd = &check_cmd_1st_stage;
      return;
    }
    if (new_command) {
      if (cmd_pos<MAX_CMD_LEN) {
        command_buf[cmd_pos++] = c; // add new byte to the current command
        if ((*check_cmd)()) // check if command complete and calls the corresponding function
          new_command = false;  // wait for next START BYTE
      } else new_command = false; // buffer full without a valid command, start over
    }
  }
  // Blink if not in address mode
  if (!address_mode) {
    if (millis()>last_blink+300) {
      digitalWrite(13,blinking);
      blinking = (blinking==HIGH) ? LOW:HIGH;
      last_blink = millis();
    }
  }
}
