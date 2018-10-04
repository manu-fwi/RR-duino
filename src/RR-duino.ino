#include "turnouts.h"
#include "sensors.h"
#include "answers.h"
#include "RRduino.h"
#include <EEPROM.h>
#include <Servo.h>

// *************** globals *******************
HardwareSerial& to_bus = Serial1;
bool save_cfg_to_eeprom = false;

// Pool of servos top be used
Servo servos[NB_SERVOS];  // servos[0] is reserved for turnout position tuning
volatile byte pulse_relay_pins[NB_SERVOS];  // pins that needs to be pulsed; MSB indicates state
byte version_nb;
bool address_mode = false;
 
byte address;               // This holds the address of this slave

// Timer2 init counter and secondary counter
// 256-96 = 160 ticks to overflow
// The timer freq with 1024 prescaler is roughly 16000
// So the overflow feq is 100Hz, just need to divide by 2 to get the 50Hz

const byte TCNT2Init = 96;
volatile byte timer2=0;

//********* ISR for the pin pulses (relay pins) ********

ISR(TIMER2_OVF_vect)
{
  TCNT2 = TCNT2Init;
  timer2++;
  if (timer2==2) {
    // Begin or finish a pulse (depending on current state):
    //   if HIGH, pulse should be ended and pin is discarded
    //   if LOW, pulse must begin
    for (byte i=1;i<NB_SERVOS;i++) {
      if (pulse_relay_pins[i]!=0) {
        if (pulse_relay_pins[i] & 0x80) {
          digitalWrite(pulse_relay_pins[i] & 0x7F,LOW);  // end the pulse
          pulse_relay_pins[i]=0;  // Discard pin
        } else {
          digitalWrite(pulse_relay_pins[i],HIGH);  // begin the pulse
          pulse_relay_pins[i] |= 0x80;             // Set the MSB to show pulse has begun
        }
      }
    }
    timer2 = 0;
  }
}

void noop() {}

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

// Config pin I/O for a turnout
void config_pins_turnout(turnout_cfg_t * turn)
{
  pinMode(turn->servo_pin, OUTPUT);
  // Set up relay pin, 0xFE means not used
  if (turn->relay_pin_1!=0xFE)
    pinMode(turn->relay_pin_1 & 0x80, OUTPUT);
  if (turn->relay_pin_2!=0xFE)
    pinMode(turn->relay_pin_2 & 0x80, OUTPUT);
}

// Config pin I/O for a sensor and also set the outputs to their last known state
void config_pins_sensor( sensor_cfg_t * sensor, bool init_state = true) 
{
  if (sensor->status & (1<< SENSOR_IO_BV))
    pinMode(sensor->sensor_pin, (sensor->status & (1 << SENSOR_PULLUP_BV)) ? INPUT_PULLUP : INPUT);
  else {
    pinMode(sensor->sensor_pin, OUTPUT);
    if (init_state) {
      if (sensor->status & 0x01)
        digitalWrite(sensor->sensor_pin, HIGH);
      else
        digitalWrite(sensor->sensor_pin, LOW);
    }
  }
}

// Load turnouts configs from EEPROM, will replace any turnout with the same subaddress
bool load_turnouts()
{
  int ee_add = 2;
  byte first = EEPROM.read(2);
  int nb=0;
  while (first!=0) {
    DEBUGLN(first);
    if (first & 0x80) {
#ifdef TURNOUT_COMB
      turn_comb_cfg_t * comb = read_cfg_comb(ee_add);
      if (comb) {
        comb->next = turn_comb_cfg_head;
        turn_comb_cfg_head = comb;
      } else return false;
#endif // TURNOUT_COMB
    } else {
      turnout_cfg_t * turn = read_cfg_turn(ee_add);
      if (turn) {
        // Add turnout so that the list of turnouts is sorted in ascending order with respect to subaddress
        turnout_cfg_t * place = find_last_turn_before(turn->subadd);
        if (place) {
          turn->next = place->next;
          place->next = turn;
        } else {
          turn->next = turnout_cfg_head;
          turnout_cfg_head = turn;
        }
        if (turn->next && (turn->next->subadd == turn->subadd)) { // delete an existing turnout with the same subaddress
          turnout_cfg_t * temp = turn->next;
          turn->next = turn->next->next;
          delete temp;
        }
        config_pins_turnout(turn);
      } else return false;
    }
    ee_add+=CFG_TURN_COMB_SIZE;
    first = EEPROM.read(ee_add);
    nb++;
  }
  DEBUG("turnouts:");
  DEBUGLN(nb);
  eeprom_turn_end = ee_add;
  return true;
}

bool load_sensors()
{
  if (address==0)
    return;
  DEBUGLN("loading sensors...");    
  eeprom_sensor_end = EEPROM.length()-CFG_SENSOR_SIZE;
  sensors_chng_state=0; // FIXME
  int ee_add = eeprom_sensor_end;
  byte first = EEPROM.read(ee_add);
  int nb=0;
  while (first!=0) {
    DEBUGLN(first);
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
      if (sensor->next && (sensor->next->subadd == sensor->subadd)) { // delete an existing sensor with the same subaddress
        sensor_cfg_t * temp = sensor->next;
        sensor->next = sensor->next->next;
        delete temp;
      }
      config_pins_sensor(sensor);
    } else return false;
    ee_add-=CFG_SENSOR_SIZE;
    first = EEPROM.read(ee_add);
    eeprom_sensor_end = ee_add;
    nb++;
  }
  DEBUG("sensors:");
  DEBUGLN(nb);
  return true;
}

void clear_eeprom(bool answer=true)
{
  DEBUGLN("wiped out");
  //EEPROM.write(0,0);  // Address is set to 0, this will prevent the whole EEPROM saving
  //address = 0;
  // Wipe turnouts and sensors out
  EEPROM.write(EE_BEGIN_TURN,0);
  EEPROM.write(EEPROM.length()-CFG_SENSOR_SIZE,0);
  if (answer)
    send_simple_answer(0);
}

void setup() {
  // Setup the pulse relay pins table
  noInterrupts();
  for (byte i=0;i<NB_SERVOS;i++)
    pulse_relay_pins[i]=0;
  interrupts();
  
  // Timer2 setting
  TCCR2B = 0x00; // No clock source (Timer/Counter stopped) 
  TCNT2 = TCNT2Init; // Register : the Timer/Counter (TCNT2) and Output Compare Register (OCR2A and OCR2B) are 8-bit
                    // Reset Timer Count
 
  TCCR2A = 0x00; // TCCR2A - Timer/Counter Control Register A
                 // All bits to zero -> Normal operation
 
  TCCR2B |= (1<<CS22)|(1<<CS21) | (1<<CS20); // Prescale 1024 (Timer/Counter started)
   
  TIMSK2 |= (1<<TOIE2); // TIMSK2 - Timer/Counter2 Interrupt Mask Register
  // Bit 0 - TOIE2: Timer/Counter2 Overflow Interrupt Enable
  
  //clear_eeprom(false);
#ifdef DEBUG
   Serial.begin(115200);
  while(!Serial);
#endif // DEBUG
  DEBUGLN("Started");
  to_bus.begin(19200);
  delay(1000);
  address = EEPROM.read(0);
  if (address==255) { // When eeprom has not been set it reads 255
    address = 0;
    version_nb = 0;
    // init EEPROM
    EEPROM.write(0,0);
    EEPROM.write(1,0);
    EEPROM.write(2,0);
    EEPROM.write(EEPROM.length()-CFG_SENSOR_SIZE,0);
  } else version_nb = EEPROM.read(1);
  if (address==0) {
    eeprom_turn_end = 2;
    return;
  }

  // Config "set address mode" pin
  pinMode(ADDRESS_MODE_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void load_cfg_from_eeprom()
{
  if (!load_turnouts() || !load_sensors())
    send_simple_answer(EEPROM_FULL);
  else
    send_simple_answer(0);
}

byte cmd_pos = 0;
bool new_command = false;
byte command_buf[MAX_CMD_LEN];

void send_simple_answer(byte err) {
  to_bus.write(0xFF);
  command_buf[0]&=~(1<<CMD_CMD_ANSWER_BV); // Unset command bit
  command_buf[2]=0x80 | err;  // add error code
  DEBUG(F("SEND SIMPLE ANSWER:"));
  DEBUGLN(sensors_chng_state);
  if ((sensors_chng_state>0) || async_head)
    command_buf[0] |= (1<<CMD_ASYNC_BV);  // Set pending async events

  for (byte i=0;i<3;i++)
    to_bus.write(command_buf[i]);
}

int write_one_turnout(byte subadd)
{
  DEBUG(F("Turnout command:"));
  DEBUGLN(subadd);
  turnout_cfg_t * turnout = find_cfg_turnout(subadd & 0x3F);
  if (!turnout) {
    DEBUGLN(F("Unknown turnout!"));
    return -UNKNOWN_DEV;
  }
  byte pos = (subadd >> SUB_VALUE_BV) & 0x01;
  // Check if the turnout is already in position
  if (pos == ((turnout->status >> TURNOUT_POS_BV) & 0x01))
    return 0;
  // Put it in "moving state" and set new position
  turnout->status |= (1 << TURNOUT_MOV_BV);
  if (pos)
    turnout->status |= 1 << TURNOUT_POS_BV;
  else
    turnout->status &= ~(1 << TURNOUT_POS_BV);
  turnout->status &= ~(1<<TURNOUT_SYNC_BV);
  if (!save_cfg_to_eeprom)
    return 0;
  // save position to eeprom
  int ee_add = ee_find_cfg_turnout(turnout->subadd);
  if (ee_add<0) { // not yet synced to eeprom, add it
    ee_add = ee_find_free_turnout();
    if (ee_add<0)
      return -EEPROM_FULL;
    save_cfg_turnout(ee_add,turnout);
  }
  else update_cfg_turnout(turnout,ee_add);
  return 0; // No error
}

int write_one_sensor(byte subadd)
{
  sensor_cfg_t * sensor;
  DEBUG(F("write sensor command:"));
  DEBUGLN(subadd);
  sensor = find_cfg_sensor(subadd & 0x3F);
  if (!sensor) {
    DEBUGLN(F("unknown sensor"));
    return -UNKNOWN_DEV;
  }
  if (sensor->status & (1 << SENSOR_IO_BV)) {
    DEBUGLN(F("Trying to write on input sensor"));
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
  sensor->status &= ~(1<<SENSOR_SYNC_BV);
  if (!save_cfg_to_eeprom)
    return 0;  // No error
  // save state to eeprom
  int ee_add = ee_find_cfg_sensor(sensor->subadd);
  if (ee_add<0) { // not yet synced to eeprom, add it
    ee_add = ee_find_free_sensor();
    if (ee_add<0)
      return -EEPROM_FULL;
  }
  update_cfg_sensor(sensor->subadd,sensor->sensor_pin,sensor->status,ee_add);
  sensor->status |= (1<<SENSOR_SYNC_BV);
  return 0; // No error  
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
  DEBUGLN(F("Writing to all turnouts"));
  byte pos=2;        // current byte in the commmand
  byte bit_index=0; // bit number in the current byte (MSB is not used for values)
  
  turnout_cfg_t * turn;
  for (turn = turnout_cfg_head;turn;turn=turn->next) {
    if (command_buf[pos] & 0x80) // this must be the end of the bits stream
      break;
    byte turn_pos = (command_buf[pos]>> bit_index) & 0x01;
    if (turn_pos != ((turn->status >> TURNOUT_POS_BV)&0x01)) {
      // Put it in "moving state" and set new position
      turn->status |= (1 << TURNOUT_MOV_BV);
      if (turn_pos)
        turn->status |= 1 << TURNOUT_POS_BV;
      else
        turn->status &= ~(1 << TURNOUT_POS_BV);
    }
    if (bit_index==6) { // next byte
      bit_index = 0;
      pos+=1;
    }
    else
      bit_index++;
  }
  if (turn) {
    DEBUGLN(F("Not all turnouts have been written to!"));
  }
  send_simple_answer(0);
}

void write_all_sensors()
{
  DEBUGLN(F("Writing to all sensors"));
  byte pos=2;        // current byte in the commmand
  byte bit_index=0; // bit number in the current byte (MSB is not used for values)
  
  sensor_cfg_t * sensor;
  for (sensor = sensor_cfg_head;sensor;sensor=sensor->next) {
    if (!(sensor->status & (1<<SENSOR_IO_BV))) { // Process only output sensors
      if (command_buf[pos] & 0x80) // this must be the end of the bits stream
        break;
     byte sensor_val = (command_buf[pos] >> SUB_VALUE_BV) & 0x01;
      if (sensor_val)
        sensor->status |= 1;
      else
        sensor->status &= 0xFE;
      if (bit_index==6) { // next byte
        bit_index = 0;
        pos+=1;
      }
      else
        bit_index++;
    }
  }
  if (sensor) {
    DEBUGLN(F("Not all turnouts have been written to!"));
  }
  send_simple_answer(0);
}

int delete_one_turnout(byte subadd)
{
  DEBUG(F("Delete Turnout command:"));
  DEBUGLN(subadd);
  // Find the previous one
  turnout_cfg_t * prec = find_last_turn_before(subadd);
  turnout_cfg_t * cfg;
  if (prec)
    cfg = prec->next;
  else
    cfg=turnout_cfg_head;
    
  if (!cfg || cfg->subadd!=subadd) {
    DEBUGLN(F("Unknown turnout!"));
    return -UNKNOWN_DEV;
  }
  //Unlink
  if (prec)
    prec->next = cfg->next;
  else
    turnout_cfg_head = cfg->next;
  delete cfg;
  if (save_cfg_to_eeprom) {
    // Delete i eeprom also
    int ee_add = ee_find_cfg_turnout(subadd);
    if (ee_add>0) // Weird if it is negative as the turnout existed, but just silently ignore this case
      EEPROM.write(ee_add, EE_FREE_TURN);
  }
  return 0; // No error
}

int delete_one_sensor(byte subadd)
{
  sensor_cfg_t * cfg,* prec;
  DEBUG(F("Delete sensor command:"));
  DEBUGLN(subadd);
  // Find the previous one
  prec = find_last_sensor_before(subadd);
  if (prec)
    cfg = prec->next;
  else
    cfg=sensor_cfg_head;

  if (!cfg || cfg->subadd != subadd) {
    DEBUGLN(F("unknown sensor"));
    return -UNKNOWN_DEV;
  }
  //Unlink
  if (prec)
    prec->next = cfg->next;
  else
    sensor_cfg_head = cfg->next;
  delete cfg;
  if (save_cfg_to_eeprom) {
    // Delete i eeprom also
    int ee_add = ee_find_cfg_sensor(subadd);
    if (ee_add>0) // Weird if it is negative as the turnout existed, but just silently ignore this case
      EEPROM.write(ee_add, EE_FREE_SENSOR);
  }
  return 0;  // No error
}

//Delete several turnouts/sensors
void delete_several()
{
  int (*func)(byte); // pointer on the func to be called
  // choose turnout/sensor according to command byte
  if (command_buf[0] & CMD_SENS_TURN_BV)
    func = &delete_one_turnout;
  else
    func = &delete_one_sensor;
    
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

// read one sensor and returns its value
// also unset the changed state if it was set and correct the sensors changed state counter
int read_one_sensor(byte subadd)
{
  sensor_cfg_t * sensor;
  DEBUG(F("read sensor command:"));
  DEBUGLN(subadd);
  sensor = find_cfg_sensor(subadd & 0x3F);
  if (!sensor) {
    DEBUGLN(F("unknown sensor"));
    return -UNKNOWN_DEV;
  }
  byte val = sensor->status & 0x1;
  if (sensor->status & (1 << SENSOR_IO_BV)) {
    // Input sensor, send last validated state

    if (sensor->status & (1 << SENSOR_CHNG_STATE_BV))
    {
      sensor->status &= ~(1 << SENSOR_CHNG_STATE_BV);
      DEBUGLN(sensor->status);     
      if (sensors_chng_state>0)
        sensors_chng_state--;
    }
  }
  return val;
}

// read one turnout and returns its value
int read_one_turnout(byte subadd)
{
  turnout_cfg_t * turn;
  DEBUG(F("read turnout command:"));
  DEBUGLN(subadd);
  turn = find_cfg_turnout(subadd & 0x3F);
  if (!turn) {
    DEBUGLN(F("unknown turnout"));
    return -UNKNOWN_DEV;
  }
  byte val = (turn->status >> TURNOUT_POS_BV) & 0x01;
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
      send_one_msg(command_buf,pos+1);
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
  DEBUGLN(F("reading all sensors"));
  // We use the command buffer as we are sending the answer right away, no need to consume even more memory
  command_buf[0] &= ~(1 << CMD_CMD_ANSWER_BV);  // It is an answer now
  command_buf[1]|= (1<<ADD_LIST_BV);
  cmd_pos = 2;  // beginning of the payload
  byte bit_pos = 0;
  command_buf[cmd_pos]=0;
  for (sensor_cfg_t * sensor=sensor_cfg_head;sensor;sensor=sensor->next) {
    if (bit_pos==7) { // go to next byte this one is full (only 7 bits are used MSB must be 0
      cmd_pos++;
      bit_pos = 0;
      command_buf[cmd_pos]=0;
    }
    if (sensor->status & 0x01)
      command_buf[cmd_pos] |= (1 << bit_pos);
    bit_pos;
  }
  command_buf[++cmd_pos]=0x80;
  send_one_msg(command_buf, cmd_pos+1);
}

void read_all_turnouts()
{
  DEBUGLN(F("reading all turnouts"));
  // We use the command buffer as we are sending the answer right away, no need to consume even more memory
  command_buf[0] &= ~(1 << CMD_CMD_ANSWER_BV);  // It is an answer now
  command_buf[1]|= (1<<ADD_LIST_BV);
  cmd_pos = 2;  // beginning of the payload
  command_buf[cmd_pos]=0;
  byte bit_pos = 0;

  for (turnout_cfg_t * turnout=turnout_cfg_head;turnout;turnout=turnout->next) {
    if (bit_pos==7) { // go to next byte this one is full (only 7 bits are used MSB must be 0
      cmd_pos++;
      bit_pos = 0;
      command_buf[cmd_pos]=0;
    }
    bool thrown = (turnout->status & (1 << TURNOUT_POS_BV))!=0;
    if ((turnout->status & (1 << TURNOUT_MOV_BV))!=0)  // It is moving so actually invert the position
      thrown = !thrown;
    if (thrown) {
      command_buf[cmd_pos] |= (1 << bit_pos);
    }
    bit_pos++;
  }
  command_buf[++cmd_pos]=0x80;
  send_one_msg(command_buf, cmd_pos+1);
}

// data points to the part of a buffer where the function puts
// the subaddress and the pin number with correct bits set (I/O, pullup,...)
// returns the number of bytes used (always 2 for sensors)
byte show_one_sensor(sensor_cfg_t * sensor, byte * data)
{
  data[0]=sensor->subadd;  
  data[1]=sensor->sensor_pin;
  DEBUG(F("SHOW ONE SENSOR "));
  DEBUG(sensor->subadd);
  DEBUG(F(" "));
  DEBUGLN(sensor->sensor_pin);
  // Set I/O bit
  if (!(sensor->status & (1 << SENSOR_IO_BV)))
    data[0] |= (1<<SUB_IODIR_BV); // output
  else { // input
    if (sensor->status & (1 << SENSOR_PULLUP_BV)) // set pullup
      data[1] |= (1 << PIN_PULLUP_BV);
  }
  return 2;
}

// data points to the part of a buffer where the function puts
// the subaddress and the servo pin number with correct bits set
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
    DEBUG(F("relay pins! = "));
    DEBUGLN(turn->relay_pin_1);
    DEBUGLN(turn->relay_pin_2);
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
    command_buf[1]|= (1<<ADD_LIST_BV); // Set the "list" bit
    if (sensor_cfg_head) 
      for (sensor_cfg_t * sensor = sensor_cfg_head;sensor;sensor=sensor->next) {
        pos += show_one_sensor(sensor,command_buf+pos);
        if ((pos+3>MAX_CMD_LEN) || !sensor->next)
        { // current command is full or we are done so queue the current answer
          byte * data = (byte*) new byte[pos+1]; // Allocate mem
          memcpy(data, command_buf, pos+1); // copy answer
          data[pos]=0x80; // Add termination
          queue_answer(data, pos+1);
          pos = 2; // reset position
        }
      }
    else {
      // Empty answer
      byte * data = (byte*) new byte[3];
      memcpy(data,command_buf,2);
      data[2]=0x80;
      queue_answer(data, 3); 
    }
  }
  send_answers(); // send them; some might not be sent (depending on the limit), they will be sent on the next show command
}

void show_turnouts_cmd(byte limit)
{
  if (!answers_head)  // if no answer is waiting this means it is the first show command
  { // build the answers
    byte pos = 2; // beginning of the payload of the command
    command_buf[0] &= ~(1 << CMD_CMD_ANSWER_BV); // Unset command bit ( this is an answer )
    command_buf[1]|= (1<<ADD_LIST_BV); // Set the "list" bit
    if (turnout_cfg_head)
      for (turnout_cfg_t * turn = turnout_cfg_head;turn;turn=turn->next) {
        pos += show_one_turnout(turn,command_buf+pos);
        if ((pos+7>MAX_CMD_LEN) || !turn->next)
        { // command is full or last turnout so queue the current answer
          byte * data = (byte*) new byte[pos+1]; // Allocate mem
          memcpy(data, command_buf, pos+1); // copy answer
          data[pos]=0x80; // Add termination no error
          queue_answer(data, pos+1);
          pos = 2; // reset position
        }
      }
    else {
      byte * data = (byte*) new byte[3];
      memcpy(data,command_buf,2);
      data[2]=0x80;
      queue_answer(data, 3);
    }
  }
  send_answers(); // send them; some might not be sent (depending on the limit), they will be sent on the next show command
}

// Read the config of a turnout from the command buf at pos
// update or create a new turnout
// returns the number of bytes read from the command buf, or a negative number in case of error
int config_one_turnout(byte pos)
{
  byte subadd = command_buf[pos] & 0x3F;
  byte cfg_size = 4;
  
  turnout_cfg_t * cfg = find_cfg_turnout(subadd);
  if (cfg) {
    DEBUGLN(F("Updating turnout"));
    // exists already, update it
    cfg->servo_pin=command_buf[pos+1];  
    cfg->straight_pos = command_buf[pos+2];
    cfg->thrown_pos = command_buf[pos+3];
    if (command_buf[pos] & (1<<SUB_RELAY_PIN_BV)) { // relay pins present
      cfg->relay_pin_1 = command_buf[pos+4];
      cfg->relay_pin_2 = command_buf[pos+5];
      DEBUG(F("relay pins = "));
      DEBUG(cfg->relay_pin_1);
      DEBUG(" ");
      DEBUGLN(cfg->relay_pin_2);
      cfg_size = 6;
    } else { // no relay pins
      cfg->relay_pin_1 = 0xFE;
      cfg->relay_pin_2 = 0xFE;
    }
    if (save_cfg_to_eeprom)
      update_cfg_turnout(cfg);
  } else {
    // Allocate new config struct and populate it
    int ee_free_slot = ee_find_free_turnout();
    if (ee_free_slot == -1) {
      DEBUGLN(F("EEPROM full"));
      return -EEPROM_FULL;
    }
    cfg = new turnout_cfg_t;
    if (!cfg) {
      DEBUGLN(F("Memory full"));
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
    DEBUG(F("New turnout, relay pins = "));
    DEBUG(cfg->relay_pin_1);
    DEBUG(" ");
    DEBUGLN(cfg->relay_pin_2);
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

    if (save_cfg_to_eeprom)
      save_cfg_turnout(ee_free_slot,cfg);
  }
  DEBUG(F("CONFIG ONE TURNOUT "));
  DEBUGLN(cfg_size);
  DEBUG(cfg->servo_pin);
  DEBUG(" ");
  DEBUG(cfg->straight_pos);
  DEBUG(" ");
  DEBUG(cfg->thrown_pos);
  DEBUG(" ");
  DEBUG(cfg->relay_pin_1);
  DEBUG(" ");
  DEBUGLN(cfg->relay_pin_2);

  config_pins_turnout(cfg);

  return cfg_size;
}

// Read the config of a sensor from the command buf at pos
// update / create a new sensor
// returns the number of bytes read from the command buf, or a negative number in case of error
int config_one_sensor(byte pos)
{
  sensor_cfg_t * cfg = find_cfg_sensor(command_buf[pos] & 0x3F);
  byte status = 0;
  // Set bits for status
  if (!(command_buf[pos] & (1<<SUB_IODIR_BV))) {
    DEBUGLN(F("input sensor cfg"));
    status |= (1<<SENSOR_IO_BV);
  }
  if (command_buf[pos+1] & (1<<PIN_PULLUP_BV)) // Check pullup
    status |= (1 << SENSOR_PULLUP_BV);
  if (cfg) {
    // this cfg exists so just update it
    cfg->sensor_pin = command_buf[pos+1] & 0x7F;
    if (save_cfg_to_eeprom) {
      update_cfg_sensor(command_buf[pos]&0x3F,cfg->sensor_pin,status);
      cfg->status=status | (1<<SENSOR_SYNC_BV);
    }
  } else {
    // Allocate new config struct and populate it
    cfg = new sensor_cfg_t;
    if (!cfg) {
      DEBUGLN(F("Memory full"));
      return -MEMORY_FULL;
    }
    // Populate cfg
    cfg->subadd = command_buf[pos] & 0x3F;
    cfg->sensor_pin = command_buf[pos+1]& 0x7F;
    cfg->status = status;
    cfg->last_time=millis();
    // add it to the list, ensuring the list stays in ascending subaddress order
    sensor_cfg_t * previous = find_last_sensor_before(cfg->subadd);
    if (previous) {
      cfg->next = previous->next;
      previous->next = cfg;
    } else {
      cfg->next = sensor_cfg_head;
      sensor_cfg_head = cfg;
    }
    // save it to eeprom
    if (save_cfg_to_eeprom) {
      int ee_add = ee_find_free_sensor();
      if (ee_add<0) {
        DEBUGLN(F("EEPROM full"));
        return -EEPROM_FULL;
      }
      save_cfg_sensor(ee_add,cfg);
    }
  }
  // config pins
  config_pins_sensor(cfg, false);
  return 2;
}

void config_several()
{
  int (*func)(byte);
  if (command_buf[0] & (1<<CMD_SENS_TURN_BV))
    func = &config_one_turnout;
  else
    func = &config_one_sensor;
  // Modify command_buf to become the answer
  for (byte pos=2;pos<cmd_pos-1;)  // go over all subadresses (the last one is 0x80 so stop before)
  {
    DEBUG(F("pos="));
    DEBUGLN(pos);
    int val = (*func)(pos);
    if (val<0) { // error
      send_simple_answer(-val);
      return;
    }
    pos += val; // advance to next config
  }
  send_simple_answer(0);  // no error occured
}

#ifdef TURNOUT_COMB
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
    if (save_cfg_to_eeprom)
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
    if (save_cfg_to_eeprom)
      save_cfg_turn_comb(ee_add,cfg);
    answer_cfg(false,false);   
  }
}
#endif

bool in_range(byte val,byte bound_1,byte bound_2)
{
  byte m = min(bound_1,bound_2);
  byte M = max(bound_1,bound_2);

  return (val>=m) && (val<=M);
}

void begin_pin_pulse(turnout_cfg_t * turn)
{
  byte pin;
  bool first = true;
  if (turn->status & (1 << TURNOUT_POS_BV)) {// which position
    pin = turn->relay_pin_2;
    first = false;
  }
  else
    pin = turn->relay_pin_1; 
  // Is it a pulsed pin
  if (pin & (1<<PIN_RELAY_PULSE_BV)) {
    noInterrupts();
    pulse_relay_pins[turn->status & 0x1F]=pin & 0x80;
    interrupts();
  } else {
    // Non pulse relay so switch one ON the other OFF
    if (first && (turn->relay_pin_2!=0xFE))
      digitalWrite(turn->relay_pin_2 & 0x80, LOW);
    else if (!first && (turn->relay_pin_1!=0xFE))
      digitalWrite(turn->relay_pin_1 & 0x80, LOW);
    if (pin != 0xFE)
      digitalWrite(pin & 0x80, HIGH);
  }
  
}

// Move all turnouts that have been activated
void process_turnouts()
{
  turnout_cfg_t * cur = turnout_cfg_head;
  for (;cur;cur = cur->next) {
    if (cur->status & (1 << TURNOUT_MOV_BV)) {
      if ((cur->status & ~0x1F)==0)  // turnout is being fine tuned, dont move it
        continue;
      int dir = (cur->thrown_pos>cur->straight_pos) ? 1 : -1;
      // check if this is a new movment, in that case, attach the servo
      if ((cur->status & 0x01F)==NO_SERVO) {
        int servo_index = find_free_servo();
        if (servo_index ==-1)
          // No free servo just return, we'll try later
          continue;
        cur->status = (cur->status & 0xE0) + (servo_index & 0x1F); // set the servo index
        // Special case: if current_pos is UNVALID_POS, the turnout has just been brought up
        // We just put it in place rapidly and pulse the relay pin if needed
        if (cur->current_pos == UNVALID_POS) {
          begin_pin_pulse(cur); // pulse the relay immediately
          // Setup the pos right before the end position
          if (cur->status & (1 << TURNOUT_POS_BV))
            cur->current_pos = cur->thrown_pos-10*dir;
          else
            cur->current_pos = cur->straight_pos+10*dir;
        } else {
          // Put the correct current position
          if (cur->status & (1 << TURNOUT_POS_BV))
            cur->current_pos = cur->straight_pos;
          else
            cur->current_pos = cur->thrown_pos;
        }
        servos[cur->status & 0x01F].write(cur->current_pos);  
        servos[cur->status & 0x01F].attach(cur->servo_pin);
      }
      else if (!in_range(cur->current_pos,cur->straight_pos,cur->thrown_pos)) {
        // Movement is done, detach the servo
        servos[cur->status & 0x1F].detach();
        cur->status = (cur->status &0xE0 & ~(1 << TURNOUT_MOV_BV)) | NO_SERVO;  // unset servo index and movement bit
        DEBUG(cur->status);
        // Queue async event
        queue_async_turnout(cur);
      }
      else {
        servos[cur->status & 0x1F].write(cur->current_pos);
        if (cur->current_pos == abs(cur->straight_pos-cur->thrown_pos)/2)
          begin_pin_pulse(cur);
      }
      // add dir to go from straight to thrown pos, substract otherwise
      DEBUGLN(cur->current_pos);
      if (cur->status & (1 << TURNOUT_POS_BV))
        cur->current_pos += dir;
      else
        cur->current_pos -= dir;
    }
  }
}

void version_cmd()
{
  byte msg[]={(1<<CMD_CFGCMD_BV)|(1 << CMD_CFG_SPECIAL_BV),address,version_nb,0x80};
  DEBUGLN(F("Version"));
  send_one_msg(msg,4);
}

void set_address(byte add)
{
  if (digitalRead(ADDRESS_MODE_PIN)==LOW) {
    DEBUG(F("Set address to:"));
    DEBUGLN(add);
    digitalWrite(13,LOW);
    address = add;
    send_simple_answer(0);
  } else DEBUGLN(F("Set address but not in address mode"));  // No reply here it was not meant for us
}

void async_cmd()
{
  DEBUGLN(F("Sending async events"));
  send_async_events();
}

// send msg on the bus
void send_one_msg(byte * msg,byte len)
{
  if ((sensors_chng_state>0) || async_head)
    msg[0] |=  (1<<CMD_ASYNC_BV);  // Set pending async events
  to_bus.write(0xFF); // Start byte
  for (byte i=0;i<len;i++)
    to_bus.write(msg[i]);
}

long unsigned last_run = 0;
bool (*check_cmd)(void)=NULL;  // function pointer to the current check command function

bool check_turnout_fine_cmd_2nd_stage()
{
  if (cmd_pos<4)
    return false;
  // Complete!
  byte subadd = command_buf[2] & 0x3F;
  turnout_cfg_t * turn = find_cfg_turnout(subadd);
  if (!turn) {
    send_simple_answer(UNKNOWN_DEV);
    return true;
  }
  if ((command_buf[3]>=10) && (command_buf[3]<=170)) { // reasonable values
    if ((turn->status & 0x1F!=0) &&(turn->status & 0x1F!=NO_SERVO)) // Check if it was attached to a servo
      servos[turn->status & 0x1F].detach();
    if (turn->status & 0x1F!=0) { // It was not already doing fine tuning so attach it to servos[0]
      turn->status &= 0xE0;   // Means: reserved for fine tuning
      servos[0].detach();
      servos[0].attach(turn->servo_pin);
    }
    servos[0].write(command_buf[3]); //set position
  } else if ((command_buf[3]==0) && (turn->status & 0x1F==0) ) // pos=0 means detach servo
    servos[0].detach();
  send_simple_answer(0);
  return true;
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
    if (command_buf[0] & (1<<CMD_SENS_TURN_BV)) { // turnout config
      //check if we have just begun a turnout config and if it is a termination (0x80)
      DEBUG(F("TURNOUT CONFIG "));
      DEBUGLN(cmd_pos);
      if (beg_turnout_cfg(cmd_pos-1) && (command_buf[cmd_pos-1]==0x80)) { //complete!
        config_several();
        return true;
      }
    } else { // sensor config
      //check if we have just begun a sensor config and if it is a termination (0x80)
      if (((cmd_pos-1)%2==0) && (command_buf[cmd_pos-1]==0x80)) { //complete!
        config_several();
        return true;
      }      
    }
    return false;
  }
  // Only one config
  if (command_buf[0] & (1<<CMD_SENS_TURN_BV)) { // turnout config
    byte cfg_size = 2 + 4;
    if (command_buf[2] & (1<<SUB_RELAY_PIN_BV))
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
  DEBUGLN("rwcmd_2");
  DEBUGLN(command_buf[0]);
  if (command_buf[0] & (1 << CMD_ALL_BV)) { // read or write ALL
    DEBUGLN("All R or W");
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
      if (command_buf[0] & (1 << CMD_SENS_TURN_BV)) {
        int err = write_one_turnout(command_buf[2]);
        send_simple_answer(-err);
      }
      else {
        int err = write_one_sensor(command_buf[2]);
        send_simple_answer(-err);
      }
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

bool check_del_cfgcmd_2nd_stage()
{
  if (cmd_pos<2)
    return false;
  if (command_buf[1] & (1 << ADD_LIST_BV)) { // List of subaddresses, terminated by 0x80
    if ((cmd_pos>=3) && (command_buf[cmd_pos-1]==0x80)) // Complete!
    {
      delete_several();
      return true;
    }
    return false;
  }
  // Read or write on one subaddress only
  if (cmd_pos == 3) { // Complete!
    if (command_buf[0] & (1 << CMD_SENS_TURN_BV)) {
      int err = delete_one_turnout(command_buf[2]);
      send_simple_answer(-err);
    }
    else {
      int err = delete_one_sensor(command_buf[2]);
      send_simple_answer(-err);
    }
    return true;
  }
  return false;
}

void save_config()
{
  int err = 0; // No error
  // Turnouts first
  turnout_cfg_t * turn = turnout_cfg_head;
  DEBUGLN(F("saveconfig, turnouts first"));
  while (turn)
  {
    if ((turn->status & (1<<TURNOUT_SYNC_BV))==0) {
      int ee_add = ee_find_cfg_turnout(turn->subadd);
      if (ee_add<0) { // new config, add it in EEPROM
        ee_add = ee_find_free_turnout();
        if (ee_add>0)
          save_cfg_turnout(ee_add, turn);
        else {
          // FIXME
          DEBUGLN(F("Full EEPROM"));
          err = EEPROM_FULL;
        }
      }
      else
        update_cfg_turnout(turn, ee_add);
    }
    turn = turn->next;
  }
  //Sensors
  sensor_cfg_t * sensor = sensor_cfg_head;
  DEBUGLN(F("Now sensors"));
  while (sensor)
  {
    if ((sensor->status & (1<<SENSOR_SYNC_BV))==0) {
      int ee_add = ee_find_cfg_sensor(sensor->subadd);
      if (ee_add<0) { // new config, add it in EEPROM
        ee_add = ee_find_free_sensor();
        if (ee_add>0)
          save_cfg_sensor(ee_add, sensor);
        else {
          // FIXME
          DEBUGLN(F("Full EEPROM"));
          err = EEPROM_FULL;
        }
      }
      else
        update_cfg_sensor(sensor->subadd, sensor->sensor_pin,sensor->status,ee_add);
    }
    sensor = sensor->next;
  }
  send_simple_answer(err);
}

// Checks if address is correct and command is complete then calls the corresponding function
// for simple commands, otherwise set the check command pointer to the next stage check function
// if command has been processed or address is not ours returns true, false otherwise
bool check_cmd_1st_stage()
{
  if (!(command_buf[0] & (1<<CMD_CMD_ANSWER_BV)))  // If this is an answer discard it
    return true;
  if (cmd_pos<=1)
    return false; // no address byte yet
  DEBUG(F("adress="));
  DEBUG(address);
  DEBUG(F(" received address="));
  DEBUGLN(command_buf[1]&0x3F);
  if (!address_mode && ((command_buf[1]&0x3F)!=address)) // check address unless we are in address mode
    return true;
  DEBUGLN(F("check cmd 1st stage C "));
  DEBUGLN(cmd_pos);

// If we are here this means command is for us and cmd_pos>=2, so we have at least command and address byte

  if (command_buf[0] & (1 << CMD_ASYNC_BV)) {
    // ASYNC command
    async_cmd();
    return true;  
  }
  if (command_buf[0] & (1 << CMD_CFGCMD_BV))  // Config commands
  {
    DEBUGLN(F("Config command"));
    if (command_buf[0] & (1 << CMD_CFG_SPECIAL_BV))  //Special config commands
    {
      DEBUGLN(F("Special config command"));
      switch ((command_buf[0] >> 4)&0x07) {
        case 0b000: //Version command
          version_cmd();
          return true;
        case 0b001:  // Set address command
          set_address(command_buf[1]);
          return true;
        case 0b010: // Store config in EEPROM
          save_cfg_to_eeprom = true;
          save_config();
          return true;
        case 0b011: // Load config from EEPROM
          load_cfg_from_eeprom();
           return true;
        case 0b100:
          // show sensors command
          show_sensors_cmd(command_buf[2]);          
          return true;
        case 0b101:
          // show turnouts command
          show_turnouts_cmd(command_buf[2]);
          return true;

        case 0b110:
          // Turnout fine tuning, set the check function accordingly
          check_cmd = & check_turnout_fine_cmd_2nd_stage;
          return false;       
        case 0b111:
          clear_eeprom();
          return true;
      }
    }
    if (command_buf[0] & (1<<CMD_CFG_DEL_BV))
      check_cmd = & check_del_cfgcmd_2nd_stage; // delete config command
    else
      check_cmd = & check_cfgcmd_2nd_stage;  // it is a normal config command set the check cmd accordingly
    return false;
  }
  DEBUGLN("fin check 1st stage");
  check_cmd = &check_rwcmd_2nd_stage;  //now set the check cmd pointer to the 2nd stage rw command
  return check_rwcmd_2nd_stage();
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
  // Blink if not in address mode
  if (!address_mode) {
    if (millis()>last_blink+300) {
      digitalWrite(13,blinking);
      blinking = (blinking==HIGH) ? LOW:HIGH;
      last_blink = millis();
    }
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
        DEBUG(c);
        DEBUG(" ");
        if ((*check_cmd)()) // check if command complete and calls the corresponding function
          new_command = false;  // wait for next START BYTE
      } else new_command = false; // buffer full without a valid command, start over
    }
  }
}
