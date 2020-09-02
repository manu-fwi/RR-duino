#include "bus_comm.h"
#include "discover.h"
#include "nodes_list.h"

byte last_discover_add=0;

void next_discover_add()
{
  // 62 is the maximum address node
  if (last_discover_add >= 62) {
    last_discover_add=63;
    return;
  }
  while (last_discover_add<=62) {
    last_discover_add++;
    node * cur;
    for (cur = nodes_head;cur && cur->address!=last_discover_add;cur=cur->next);
    if (!cur)
      break;
  }
}

// Check if there is a node responding at the address
// returns true on success or false otherwise
bool check_address(byte address)
{
  int version = ask_version(address);
  if (version<0)
    // Fow now any error is interpreted as "no node at this address"
    return false;
  return true;
}
