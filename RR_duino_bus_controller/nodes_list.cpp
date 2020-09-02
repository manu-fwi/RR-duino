#include "nodes_list.h"

node * nodes_head = NULL;
byte online_nodes = 0;

// return the node or the last one before if not present
node * find_node_from_add(byte address) {
  node * previous = NULL;
  node * current = nodes_head;
  
  while(current && (current->address<=address)) {
    previous = current;
    current = current->next;
  }
  return previous;
}

// return the first node (from beginning onwards) that is in state "state" for more than timeout
node * first_timedout_node(byte state,unsigned long timeout,node * beginning)
{
  for (node * current = beginning;current;current=current->next) {
    if ((current->state == state) && (millis()-current->last_ping>timeout))
      return current;
  }
  return NULL;
}

String bytes_to_wire(byte bits[],int len)
{
  String result = "";

  for (int i = 0;i<len;i++) {
    result+=" "+String(bits[i],HEX);
  }
  return result;
}
