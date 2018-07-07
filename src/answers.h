#ifndef ANSWERS_H
#define ANSWERS_H

typedef struct answer_t{
  char * answer_str;
  answer_t * next;
};

extern answer_t * answers_head;

void send_pending_answers(unsigned limit);
void queue_answer(char * txt);

#endif // ANSWERS_H
