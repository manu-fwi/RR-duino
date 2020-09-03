#ifndef BUS_COMM_H
#define BUS_COMM_H
#include <Arduino.h>
#include "nodes_list.h"

// Errors
#define NOT_AN_ANSWER 1     // We were waiting for an answer and its not
#define NOT_SAME_ADD  2     // We were waiting an answer from another address
#define NOT_SAME_COMMAND 3  // The answer does not correspond to the command we have sent
#define ANSWER_TOO_LONG 4   // The answer overfilled the buffer
#define ANSWER_ERR_TIMEOUT 5 // Timed out waiting for the answer
#define ANSWER_ERROR 0x10   // The answer signals an error in the lower nibble

// Constants
#define MAX_CMD_LEN 63
#define ANSWER_TIMEOUT 100 // In ms

// Variables needed by other modules
extern byte command,address,buf[];

// Functions
void set_data_dir(bool write);
void send_one_msg(byte * msg,byte len);
int answer_from_bus(void (*func)()); // return 0 if incomplete, 1 if valid and complete, <0 if error
int ask_version(byte address); // return version (>=0) or error (<0)
int store_eeprom(byte address);
int load_eeprom(byte address);
int show_tables_cmd(byte address);  // Send show table cmd, process the answer and insert a new node if everything went fine
int read_all(node * new_node,bool turnouts = false); // read all states of sensors/turnouts
int async_read(node * node_to_ping);   // used to check if node has some changes to report

#endif // BUS_COMM_H
