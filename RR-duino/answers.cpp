#include "sensors.h"
#include "turnouts.h"
#include "answers.h"
#include "RRduino.h"
#include <Arduino.h>

// List of answers to the last "list" command
answer_t * answers_head=0;

//List of async events related to turnouts
answer_t * async_head = 0;

void send_answers()
{
  DEBUGLN(F("Send answers "));

  if (answers_head) {
    if ((sensors_chng_state) || async_head) { // any input changed or async events waiting?
      answers_head->data[0] |= (1 << CMD_ASYNC_BV);
    }
    DEBUGLN(F("SENDING ANSWER"));
    set_data_dir();
    to_bus.write(0xFF); // Start byte
    for (byte j=0;j<answers_head->len;j++)
      to_bus.write(answers_head->data[j]);
    to_bus.flush();
    set_data_dir(false);
    delete answers_head->data;
    answer_t * temp = answers_head;
    answers_head = answers_head->next;
    delete temp;
  }
}

void send_async_events()
{
  bool empty_answer=true;
  // First send any turnout async answer
  if (async_head) {
    DEBUGLN(F("ASYNC EVENTS, NON SENSOR RELATED"));
    if (!(async_head->next)) { // last answer to be sent
      // We need to check if we set the other answers pending (in case of async events waiting)
      if (sensors_chng_state) { // any input changed or async events waiting?
        async_head->data[0] |= (1 << CMD_PEND_ANSWERS_BV);
      }
      async_head->data[async_head->len++]=0x80;// we need to add termination to the last
    }
    set_data_dir();
    to_bus.write(0xFF);
    for (byte j=0;j<async_head->len;j++)
      to_bus.write(async_head->data[j]);
    to_bus.flush();
    set_data_dir(false);
    delete async_head->data;
    answer_t * temp = async_head;
    async_head = async_head->next;
    delete temp;
  } else {
    // Now send all sensors state changes
    sensor_cfg_t * sens = sensor_cfg_head;
    byte * data = new byte[MAX_CMD_LEN];
    data[0]=(1<<CMD_ASYNC_BV);  // as a read sensor command, async bit set
    data[1]=address | (1 << ADD_LIST_BV);  // Its a list
    byte len=2;
    DEBUG(F("SENSORS ASYNCS:"));
    DEBUGLN(sensors_chng_state);
  
    while (sensors_chng_state && sens) {
      DEBUG(F("sensor change state="));
      DEBUG(sens->subadd);
      DEBUG(" ");
      DEBUGLN(sensors_chng_state);
      if (sens->status & (1 << SENSOR_IO_BV)) {
        if (sens->status & (1<<SENSOR_CHNG_STATE_BV)) {
          sensors_chng_state--;
          empty_answer = false;
          sens->status &= ~(1<<SENSOR_CHNG_STATE_BV); // Reset changed state
          // Input sensor, send last validated state
          data[len++]=sens->subadd | ((sens->status & 0x01) << SUB_VALUE_BV);
          if ((len > MAX_CMD_LEN-2) || !sensors_chng_state) { // no more space or last change
            if (sensors_chng_state)
              data[0] |= (1 << CMD_PEND_ANSWERS_BV); // more to come
            data[len++]=0x80;  // Indicates last subaddress
            set_data_dir();
            to_bus.write(0xFF); // Start byte
            for (byte j=0;j<len;j++) {
              to_bus.write(data[j]);
            }
            to_bus.flush();
            set_data_dir(false);
            break; // break, we will go on next time
          }
        }
      }
      sens = sens->next;  // Next sensor
    }
    if (empty_answer) { // in  case no event was to be reported, send an empty answer
      data[0]= 0b00000100; // nothing to be reported
      data[1]=address | (1<<ADD_LIST_BV);
      data[2]=0x80;
      set_data_dir();
      to_bus.write(0xFF); // Start byte
      for (byte j=0;j<3;j++) {
        to_bus.write(data[j]);
      }
      to_bus.flush();
      set_data_dir(false);
    }
    delete data;    
  }
}

// Queue a new answer to a list of answers
// The answer that was last is modified to signal that now there is another answer after it
void queue_answer(byte * data, byte len)
{
  DEBUG(F("Queueing answer of length "));
  DEBUGLN(len);
  answer_t * answ = new struct answer_t;
  answ->data = data;
  answ->len = len;
  answ->next = NULL;  // last one in the list
  if (answers_head) {
    answer_t * p =answers_head;
    for (;p->next;p=p->next);  // Find the last of the list
    p->next =answ;  //link to the new answer
    p->data[0] |= (1 << CMD_PEND_ANSWERS_BV); // The one before the last now says another is waiting after it
  }
  else answers_head = answ;
}

// Queue a new turnout async event
// Merge with current list or create a new one if full
void queue_async_turnout(turnout_cfg_t * turnout)
{
  answer_t * answ = async_head;
  for (;answ && answ->next;answ=answ->next);  // Find the last of the list
  DEBUG(F("QUEUE ASYNC TURNOUT="));
  DEBUG(turnout->subadd);
  DEBUG(F(",pos="));
  DEBUG(turnout->current_pos);
  DEBUG(F(",status="));
  DEBUGLN(turnout->status);
  if (!answ || (answ->len>MAX_CMD_LEN-2)) {
    // empty list or last answer is full (we need 2 bytes, 1 for the turnout subadd plus the end list byte (0x80)
    // create a new answer
    answer_t * p = new struct answer_t;
    p->data = new byte[MAX_CMD_LEN];
    p->data[0]=(1<<CMD_ASYNC_BV) | (1<<CMD_SENS_TURN_BV);  // as a read command answer with async bit set
    p->data[1]=address | (1 << ADD_LIST_BV);
    p->len = 2;
    p->next = NULL;
    if (answ) {
      answ->next = p;
      answ->data[answ->len++]=0x80; // Marks the last subadd
      answ->data[0] |= (1 << CMD_PEND_ANSWERS_BV); // set pending answers bit
    }
    else
      async_head = p;
    answ = p;
  }
  if (turnout->status & (1 << TURNOUT_POS_BV))
    answ->data[answ->len++]=turnout->subadd | (1<<SUB_VALUE_BV); // Add turnout subadd with value
  else
    answ->data[answ->len++]=turnout->subadd;
}
