#include "config.h"
#include "bus_comm.h"
#include "RR_duino_bus_controller.h"
#include "nodes_list.h"
#include "Arduino.h"

// Variables to receive and send commands
byte cmd_pos = 0;
bool new_answer = false;
byte command,address, buf[MAX_CMD_LEN];   // Command and address of the command we have sent, buf is used to store the full command and the answer

// Set data direction
void set_data_dir(bool write=true)
{
  if (DATA_DIR_PIN!=255) {
    if (write)
      digitalWrite(DATA_DIR_PIN,HIGH);
    else
      digitalWrite(DATA_DIR_PIN,LOW);
  }
}

// send msg on the bus
void send_one_msg(byte * msg,byte len)
{
  DEBUGLN("Sending message");
  DEBUG("  ");
  set_data_dir();
  TO_BUS.write(0xFF); // Start byte
  // Make sure we store command and address to check answer to this later
  command = msg[0];
  address = msg[1];
  for (byte i=0;i<len;i++) {
    TO_BUS.write(msg[i]);
    DEBUG(msg[i]);
    DEBUG("*");
  }
  DEBUGLN("");
  TO_BUS.flush();
  set_data_dir(false);
}

/* Treat incoming chars, check correctness with respect to the command we sent before
 *  returns: 
 *    0 if the answer is not yet complete
 *    1 if the answer is valid and complete
 *    <0 on errors
 */
int answer_from_bus_step()
{
  int error = 0;
  if (TO_BUS.available()) {
    byte c = TO_BUS.read();
    if (not new_answer) {
      if (c==0xFF) {
        //begin new answer
        new_answer = true;
        cmd_pos = 0;
      }
      return 0;
    }
    if (cmd_pos<MAX_CMD_LEN) {
      if (cmd_pos==0) {
        //Check if it is an answer and it corresponds the command we have sent
        if ((c&0x01)!=0)
          error = -NOT_AN_ANSWER;
        else if ((command & (1<<CMD_ASYNC_BV))==0) { // Only check if not an async command
          if ((command & 0xF8)!=(c & 0xF8))
            error = -NOT_SAME_COMMAND;                      
        }
      }
      else if (cmd_pos==1)
      {
        //Check address
        if ((c&0x3F) != (address&0x3F))
          error = -NOT_SAME_ADD;
      }
      if (error<0) {
        // Bad answer reset  everything
        new_answer = false;
        cmd_pos = 0;
        return error;
      }
      // Add byte to buffer
      buf[cmd_pos++] = c;
      DEBUG(c);
      DEBUG("x");
      // Check if we reached the end of the answer
      if ((cmd_pos>2) && ((buf[cmd_pos-1]&0x80)!=0 ))
      {
        // Check if the replying device signals an error
        if ((buf[cmd_pos-1]&0x0F)!=0 )
          error = -(ANSWER_ERROR+(buf[cmd_pos-1]&0x0F));
        else
          error = 1; // Complete and valid
        new_answer = false;
      }
    } else {
      // Answer longer than our buffer, weird, discard it
      cmd_pos = 0;
      new_answer = false;
      error = -ANSWER_TOO_LONG;
    }
  }
  return error;
}

// Task is used to do something as serial bus is slow
// NULL means do nothing
int answer_from_bus(void (*task)())
{
  DEBUGLN("ANSWER:");
  DEBUG("  ");
  int res = 0;
  new_answer = false;
  unsigned long beg = millis();
  while ((res==0) && (millis()-beg<ANSWER_TIMEOUT)) {
    res = answer_from_bus_step();
    if (task)
      task();
  }
  if (millis()-beg>=ANSWER_TIMEOUT) {
          DEBUGLN("Answer timed out!");
    return -ANSWER_ERR_TIMEOUT;
  }
  DEBUGLN();
  return res;
}

int ask_version(byte address)
{
  byte cmd[]={ 1 << CMD_CFGCMD_BV | 1 << CMD_CFG_SPECIAL_BV | 1 << CMD_CMD_ANSWER_BV,address};
  send_one_msg(cmd,2);
  int res = answer_from_bus(NULL);
  if (res<0)
    return res; // return error
  return buf[2]; // return version
}

int store_eeprom(byte address)
{
  byte cmd[]={ 1 << CMD_CFGCMD_BV | 0b010 << CMD_CFG_SPECIAL_EEPROM_BV | 1 << CMD_CFG_SPECIAL_BV | 1 << CMD_CMD_ANSWER_BV,address};
  send_one_msg(cmd,2);
  int res = answer_from_bus(NULL);
  return res;
}

int load_eeprom(byte address)
{
  byte cmd[]={ 1 << CMD_CFGCMD_BV | 0b011 << CMD_CFG_SPECIAL_EEPROM_BV | 1 << CMD_CFG_SPECIAL_BV | 1 << CMD_CMD_ANSWER_BV,address};
  send_one_msg(cmd,2);
  int res = answer_from_bus(NULL);
  return res;
}

byte nb_bits_set(byte bits_array[],byte len)
{
  byte res = 0;
  byte bit_pos=0;
  byte index=0;
  while (index<len) {
    if ((bits_array[index] & (1 << bit_pos))!=0)
      res++;
    bit_pos++;
    if (bit_pos == 7) {
      bit_pos = 0;
      index++;
    }
  }
  return res;
}

int show_tables_cmd(byte address)
{
  byte cmd[]={ 1 << CMD_CFGCMD_BV | 0b01000000 | 1 << CMD_CFG_SPECIAL_BV | 1 << CMD_CMD_ANSWER_BV,
              address | (1 << ADD_TABLE_BV)}; // Set table show command

  send_one_msg(cmd,2);
  int res;
  node * new_node = new node;

  // Set sensors arrays to empty
  memset(new_node->input_sensors,0,sizeof(new_node->input_sensors));
  memset(new_node->output_sensors,0,sizeof(new_node->output_sensors));
  memset(new_node->turnouts,0,sizeof(new_node->turnouts));
  
  res = answer_from_bus(declare_new_node_to_server);
  if (res<0) {
    delete new_node;
    return res;
  }

  // Copy the 2 bits tables (input and output sensors
  memcpy(new_node->input_sensors,buf+2,9);
  memcpy(new_node->output_sensors,buf+11,9);
  
  // Do turnouts now
  cmd[0]|=1 << CMD_SENS_TURN_BV;
  send_one_msg(cmd,2);
  res = answer_from_bus(declare_new_node_to_server);
  if (res<0) {
    delete new_node;
    return res;
  }

  // Copy the turnouts bits tables
  memcpy(new_node->turnouts,buf+2,9);
  // compute nb of sensors (total I+O) and number of turnouts
  new_node->nb_turnouts = nb_bits_set(new_node->turnouts,9);
  new_node->nb_sensors = nb_bits_set(new_node->input_sensors,9)+nb_bits_set(new_node->output_sensors,9);
  // Populate node members
  new_node->address = address;
  new_node->state = NEW_NODE;
  new_node->last_ping = 0;
  // Insert it into the nodes list
  node * prev = find_node_from_add(address);
  if (prev) {
    prev->next = new_node;
    new_node->next=prev->next;
  }
  else {
    new_node->next = nodes_head;
    nodes_head = new_node;
  }
  return res;
}

int read_all(node * new_node,bool turnout)
{
  byte cmd[]={ 1 << CMD_ALL_BV | 1 << CMD_CMD_ANSWER_BV, new_node->address}; // read all command

  if (turnout)
    cmd[0]|=1<<CMD_SENS_TURN_BV;  // for turnouts

  send_one_msg(cmd,2);
  // Get answer
  int res = answer_from_bus(declare_new_node_to_server);
  if (res<0)
    return res;
  byte pos = 0;
  if (turnout)
    memset(new_node->turnouts_states,0,sizeof(new_node->turnouts_states));
  else
    memset(new_node->sensors_states,0,sizeof(new_node->sensors_states));
  while ((buf[2+pos] & 0x80)==0) {
    if (turnout)
      new_node->turnouts_states[pos]=buf[2+pos];
    else
      new_node->sensors_states[pos]=buf[2+pos];
    pos++;

  }

  return 0;
}

int async_read(node * node_to_ping)   // used to check if node has some changes to report
{
  byte cmd[]={1 << CMD_CMD_ANSWER_BV | 1 << CMD_ASYNC_BV,node_to_ping->address};
  send_one_msg(cmd,2);
  // Get answer
  return answer_from_bus(declare_new_node_to_server);
}
