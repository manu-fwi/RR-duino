#ifndef ANSWERS_H
#define ANSWERS_H

typedef struct answer_t{
  byte * data;
  byte len;
  answer_t * next;
};

extern answer_t * answers_head;
extern answer_t * async_head;

void send_answers();
void send_async_events();
void queue_answer(byte * data, byte len);
void queue_async_turnout(turnout_cfg_t * turnout);

#endif // ANSWERS_H
