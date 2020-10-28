#ifndef DISCOVER_H
#define DISCOVER_H

#include <Arduino.h>

extern byte last_discover_add;
extern bool auto_discover;

void next_discover_add();

#endif // DISCOVER_H
