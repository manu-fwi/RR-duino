#include "sensors.h"
#include "answers.h"
#include "RRduino.h"
#include <Arduino.h>

answer_t * answers_head=0;

void send_pending_answers(unsigned limit)
{
  char * str = new char[15];
  unsigned i=0;
  if (limit==0)
    limit = 1000;
  answer_t * answer = answers_head;
  // First send any turnout async answer
  while ((i<limit) && answer) {
    to_bus.println(answer->answer_str);
    i++;
    delete answer->answer_str;
    answer_t * temp = answer;
    answer = answer->next;
    delete temp;
  }
  sensor_cfg_t * sens = sensor_cfg_head;
  while (sens) {
    if (sens->status & (1 << SENSOR_BV_IO)) {
      if (sens->status & (1<<SENSOR_BV_CHNG_STATE)) {
        sensors_chng_state--;
        // Input sensor, send last validated state
        byte val = sens->status & 0x1;
        snprintf(str,15,"<S %d %d>",sens->subadd,val);
        to_bus.println(str);
      }
    }  
    sens = sens->next;
  }
  if (answer || sensors_chng_state)
    to_bus.println("<AE+>"); // Marks the end of this batch of answers, but signal that there are more
  else
    to_bus.println("<AE>"); // Marks the end of this batch of answers, and there is none left
  delete[] str;
}

void queue_answer(char * txt)
{
  answer_t * answ = new struct answer_t;
  answ->answer_str = new char[strlen(txt)+1];
  strncpy(answ->answer_str, txt, strlen(txt));
  answ->answer_str[strlen(txt)]='\0';
  answ->next = answers_head;
  answers_head = answ;
}

