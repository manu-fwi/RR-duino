#ifndef RR_DUINO_BUS_CONTROLLER_H
#define RR_DUINO_BUS_CONTROLLER_H

#include <ESP8266WiFi.h>
#include "config.h"

// Defines for the protocol
#define START_BYTE 0xFF
#define TIME_OUT 200  // ms
#define PING_TOUT 500 // ms
#define DEAD_NODE_PERIOD 3000 // in ms, time between trials to wake a dead node up
#define SERVER_READ_TOUT 2 // in ms, timeout for reading characters from the server
#define SERVER_CONN_ANSWER_TOUT 200 // in ms, timeout when waiting for a server answer after the initial connection

// Define for the command bits positions
#define CMD_CFGCMD_BV 7
#define CMD_RWDIR_BV 5
#define CMD_ALL_BV 6
#define CMD_SENS_TURN_BV 4
#define CMD_CFG_DEL_BV 5
#define CMD_CFG_SPECIAL_BV 3
#define CMD_CFG_SPECIAL_EEPROM_BV 4
#define CMD_ASYNC_BV 2
#define CMD_PEND_ANSWERS_BV 1
#define CMD_CMD_ANSWER_BV 0

// Define for the address bits positions
#define ADD_TABLE_BV 7
#define ADD_LIST_BV 6

// Define for the subaddress bits positions
#define SUB_LAST_BV 7
#define SUB_IODIR_BV 6
#define SUB_VALUE_BV 6
#define SUB_RELAY_PIN_BV 6

// Define for the pin bits positions
#define PIN_PULLUP_BV 7
#define PIN_RELAY_PULSE_BV 7

// Server send
#define SERVER_SEND_TOUT 5 //ms


// Functions
void declare_new_node_to_server();  // Traverse the nodes list and send a NEW NODE message for the first ONLINE node
void noop();
#endif //RR_DUINO_BUS_CONTROLLER_H
