#include "sensors.h"
#include "turnouts.h"
#include "answers.h"
#include "RRduino.h"
#include <Arduino.h>

// List of answers to the last "list" command
answer_t * answers_head=0;

//List of async events related to turnouts
answer_t * async_head = 0;

void send_answers(unsigned limit)
{
  unsigned i=0;
  if (limit==0)
    limit = 1000;
  // First send any turnout async answer
  while ((i<limit) && answers_head) {
    if (!(answers_head->next)) { // last answer to be sent
      // We need to check if we set the other answers pending (in case of async events waiting)
      if ((sensors_chng_state) || async_head) { // any input changed or async events waiting?
        answers_head->data[0] |= (1 << CMD_PEND_ANSWERS_BV) | (1 << CMD_ASYNC_BV);
      }
    }
    to_bus.write(0xFF); // Start byte
    for (byte j=0;j<answers_head->len;j++)
      to_bus.write(answers_head->data[j]);
    i++;
    delete answers_head->data;
    answer_t * temp = answers_head;
    answers_head = answers_head->next;
    delete temp;
  }
}

void send_async_events(unsigned limit)
{
  unsigned i=0;
  if (limit==0)
    limit = 1000;
  // First send any turnout async answer
  while ((i<limit) && async_head) {
    if (!(async_head->next)) { // last answer to be sent
      // We need to check if we set the other answers pending (in case of async events waiting)
      if (sensors_chng_state) { // any input changed or async events waiting?
        async_head->data[0] |= (1 << CMD_PEND_ANSWERS_BV) | (1 << CMD_ASYNC_BV);
      }
    }
    for (byte j=0;j<async_head->len;j++)
      to_bus.write(async_head->data[j]);
    i++;
    delete async_head->data;
    answer_t * temp = async_head;
    async_head = async_head->next;
    delete temp;
  }
  // Now send all sensors state changes
  sensor_cfg_t * sens = sensor_cfg_head;
  byte * data = new byte[MAX_CMD_LEN];
  data[0]=(1<<CMD_ASYNC_BV);  // as a read command, async bit set
  data[1]=address | (1 << ADD_LIST_BV);  // Its a list
  byte len=2;
  while ((i<limit) && sensors_chng_state && sens) {
    if (sens->status & (1 << SENSOR_BV_IO)) {
      if (sens->status & (1<<SENSOR_BV_CHNG_STATE)) {
        sensors_chng_state--;
        // Input sensor, send last validated state
        data[len++]=sens->subadd | ((sens->status & 0x01) << SUB_VALUE_BV);
        if (len > MAX_CMD_LEN-2) { // no more space
          if (sensors_chng_state)
            data[0] |= (1 << CMD_PEND_ANSWERS_BV); // more to come
          data[len]=0x80;  // Indicates last subaddress
          to_bus.write(0xFF); // Start byte
          for (byte j=0;j<len;j++)
            to_bus.write(data[j]);
          i++;
          len = 2;
        }
      }
    }
    sens = sens->next;  // Next sensor
  }
}

// Queue a new answer to a list of answers
// The answer that was last is modified to signal that now there is another answer after it
void queue_answer(answer_t * head, byte * data, byte len)
{
  answer_t * answ = new struct answer_t;
  answ->data = data;
  answ->next = NULL;  // last one in the list
  if (head) {
    answer_t * p =head;
    for (;p->next;p=p->next);  // Find the last of the list
    p->next =answ;  //link to the new answer
    p->data[0] |= (1 << CMD_PEND_ANSWERS_BV); // The one before the last now says another is waiting after it
  }
  else head = answ;
}

// Queue a new turnout async event
// Merge with current list or create a new one if full
void queue_async_turnout(turnout_cfg_t * turnout)
{
  answer_t * answ = async_head;
  for (;answ && answ->next;answ=answ->next);  // Find the last of the list
  
  if (!answ || (answ->len>MAX_CMD_LEN-2)) { // empty list or last answer is full
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
  if (turnout->status & (1 << TURNOUT_BV_POS))
    answ->data[answ->len++]=turnout->subadd | (1<<SUB_VALUE_BV); // Add turnout subadd with value
  else
    answ->data[answ->len++]=turnout->subadd;
}
