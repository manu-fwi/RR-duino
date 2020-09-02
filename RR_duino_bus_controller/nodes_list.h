#ifndef NODES_LIST
#define NODES_LIST

#include <Arduino.h>
#define TO_DISCOVER 1 // node to try
#define NEW_NODE    2 // Node configuration is complete. Needs to read all sensors/turnouts to be declared ONLINE
#define ONLINE_NODE 3 // Operational (config and sensors/turnouts states are read)
#define CONFIRMED 4   // Operational and known to the server (normal state)
#define DEAD_NODE 10  // This node has timed out on last command

// Types

struct node {
  byte address;
  byte state;
  unsigned long last_ping;
// Binary arrays (9 bytes => 63 bits, MSB are not used, always set to 0).
// Each set bit indicates that the corresponding subaddress is taken by an output or input sensor or a turnout
  byte input_sensors[9];
  byte output_sensors[9];
  byte turnouts[9];
  byte nb_sensors;
  byte sensors_states[9];
  byte nb_turnouts;
  byte turnouts_states[9];
  node * next;
};

// Types

extern node * nodes_head;
extern byte online_nodes;

// Functions
node * find_node_from_add(byte address);
node * first_timedout_node(byte state,unsigned long timeout,node * beginning);
String bytes_to_wire(byte bits[],int len);

#endif // NODES_LIST
