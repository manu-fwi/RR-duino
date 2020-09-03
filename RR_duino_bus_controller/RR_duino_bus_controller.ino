#include "RR_duino_bus_controller.h"
#include "bus_comm.h"
#include "nodes_list.h"
#include "config.h"
#include "discover.h"
#include <ESP8266WiFi.h>

/*
 * Arduino programs that handles jmri-rrduino server requests and controls the RR_duino nodes bus (RS485)
 *
 * When it connects to the server it announces itself by
 * RRDUINO-BUS xx
 * xx is the bus number (to distinguish between several busses)
 * After this the server is able to send the requests to the corresponding bus via this controller
 *
 * When a new node is discovered:
 * NEW-NODE:address, x x x x x x x x x, x x x x x x x x x, x x x x x x x x x , x x ... x , x x ... x
 * where each "x" is a byte (0-127, MSB is always 0) indicating the subaddresses taken by input sensors then the output sensors
 * and finally the turnouts for the first three series of bytes
 * then the last two indicate the sensors states (again 0-127, MSB is always 0) and the turnouts states (the number of these last
 * two series depend on the numbers of sensors and turnouts). Each bit (minus the MSB, always 0) correspond to the state of one sensor
 * or turnout in ascending subaddress order: first bit (bit 0 of byte 0) is the state of the sensor of lowest subaddress...
 * Note that the sensors states are input and outputs mixed.
 * each bit in each integer is set to indicate that there is a sensor at the corresponding address
 * for example:
 * NEW-NODE:3, 6 64 0 0 0 0 0 0 0, 24 0 0 0 0 0 0 0 0, 0 65 0 1 0 0 0 0 0, 18, 2
 * Indicate that a new node with address 3 has been discovered and it has 3 input sensors of subaddresses: 2,3,14 as
 * these are the bits that are set (it is LSB first, that is the first bit of the first byte corresponds to subaddress 1
 * the third bit of the 4th byte corresponds to subaddress 24), output sensors of subaddresses 4 and 5 and turnouts of subaddresses
 * 8, 13 and 22. Initial states are: for sensors (inputs/outputs are mixed) (subaddress, value) (2,0) (3,1) (4,0) (5,0) (14,1)
 * and for turnouts (2,0) (3,1) (14,0)
 * This is used to query the initial state of all sensors (so we use the read all command) at startup to have a coherent view
 * of turnouts/sensors
 * 
 * Numbering scheme: in RRDuino sensors (output or inputs) can have subaddresse from 1 to 62, same for turnouts (sensors and turnouts
 * are independent)
 * Mapping to jmri:
 * - input sensors are directly mapped to sensors so RRduio sensor of subaddress 13 is ITRSbb:nn:13 in jmri
 * where bb indicates the bus number in decimal, nn the node number on the bus in decimal (the bus number will be stripped out by
 * the server)
 * - output sensors in RRduino world are mapped to turnouts in jmri but to avoid conflicts an output sensor of subaddress 14 in RRduino
 * is mapped to turnout ITRTbb:nn:114 (100 is added to the subadress) the number 1-99 are reserved for "real" turnouts see below. As
 * output sensors in RRduino can be read (this will be moslty interesting on startup as their last state is restored thanks to the eeprom
 * they can be set as turnouts with feedback, in that case the corresponding sensor will be ISRSbb:nn:213 (the sensor subaddress + 200)
 * - "real" turnouts: a turnout in jmri which represents a turnout in RRduino is mapped directly, that is turnout ITRTbb:nn:56 corresponds
 * turnout of subaddress 56 in RRduino. BUT the turnouts have feedback sensor in JMRI and this sensor will be ISRSbb:nn:156 (subaddress of
 * the turnout + 100)
 * Server commands:
 * 
 * ITRT12:6,1
 * Set the turnout of subadddress 6 of node of address 12 to thrown (the bus address is stripped off by the server, so the real command
 * coming out of JMRI via the jython script is ITRT2:12:6,1 where 2 is the bus number in this example)
 * The prefix ITRT is for turnouts
 *
 * message to the server (to indicate sensor changes, meaning input sensor change or output sensor or turnout change):
 * ISRS4:123,0  for a turnout of subaddress 23 being closed from a node of address 4
 * ISRS5:241,1  for an output sensor being HIGH (subaddress is 41 on node of address 5)
 */

// Global variables

// Connection to the server
const char * server_ip = SERVER_IP;
const unsigned int server_port = SERVER_PORT;
WiFiClient client,debug_client;
// Current server command
String server_cmd;
unsigned long last_server_send = 0;

// Led blinking
boolean state=true; // blinking the LED
unsigned long blink_time = 0;
unsigned long blink_delay = 1000; // Normal operation: blink once/second

// Command/answer
bool waiting_for_an_answer=false;
unsigned long last_command = 0;

void noop() {}

void setup() {
  // put your setup code here, to run once:
  TO_BUS.begin(SERIAL_SPEED);
  WiFi.begin(WIFISSID, WIFIPASS);
  state = true;
  digitalWrite(13,state?HIGH:LOW);
  // Connect to wifi
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    state = !state;
    digitalWrite(13,state?HIGH:LOW);
  }
  unsigned long beg = millis();
  while (!debug_client.connected() && (millis()-beg<5000)) {
    debug_client.connect("192.168.0.22",50000);
    delay(1000);
  }
  DEBUGLN("Debug Working");
  digitalWrite(13,HIGH);
  state = true;
  blink_time = millis();
}

void connect_to_server()
{
  DEBUGLN("Trying to connect to server");
  client.connect(server_ip,server_port);
  // Send bus number
  if (client.connected())
  {
    client.print("RRDUINO-BUS ");
    client.println(BUS_NUMBER);
  }
  else DEBUGLN("FAILED");
}

void process_server_cmd(String cmd)
{
  DEBUG("Cmd from server:");
  DEBUGLN(cmd);
  if (!cmd.startsWith("ITRT") && !cmd.startsWith("ISRS")) // test prefix
  {
    DEBUG("Bad prefix in command from server");
    return;
  }
  if (cmd.charAt(4)=='S')  // sensor
  {
    DEBUGLN("Got a sensor command from server");
    //Nothing to do, should not happen
    return;
  }
  else // Turnout
  {
    cmd = cmd.substring(4); // Trim the prefix
    int address=cmd.toInt(); // get node address
    if ((address<1) || (address>62))
    {
      DEBUGLN("Invalid address in command from server");
      return;
    }
    int colon = cmd.indexOf(':');
    if (colon==-1)
    {
      DEBUGLN("Missing "":"" in command from server");
      return;
    }
    node * dest = find_node_from_add(address);
    if (!dest)
    {
      DEBUG("Unknown node address ");
      DEBUG(address);
      DEBUGLN(" in command from server");
      return;
    }
    cmd = cmd.substring(colon+1);
    int comma = cmd.indexOf(',');
    if (comma==-1)
    {
      DEBUGLN("Missing comma in turnout command from server");
      return;
    }
    int subaddress = cmd.toInt();
    // FIXME check if this turnout exists
    int value = int(cmd.charAt(comma+1))-int('0'); // value should be 0 or 1
    if ((value != 0) && (value!=1))
    {
      DEBUGLN("Invalid value in command from server");
      return;
    }
    bool is_sensor = false;
    if (subaddress>100)
    {
      subaddress-=100; // It is an output sensor so get the real subaddress
      is_sensor = true;
    }
    if ((subaddress<1) || (subaddress>62))
    {
      DEBUGLN("Invalid subaddress in command from server");
      return;
    }
    // OK everything checked lets send the command on the RRduino bus
    DEBUG("Set turnout add=");
    DEBUG(address);
    DEBUG(" subadd=");
    DEBUGLN(subaddress);
    byte bus_cmd[]={ 1 << CMD_RWDIR_BV | 1,(byte)address,(byte)subaddress}; // Write command
    if (!is_sensor)
      bus_cmd[0]|=1 << CMD_SENS_TURN_BV; // It is a turnout
    send_one_msg(bus_cmd,3);
    int res = answer_from_bus(declare_new_node_to_server);
    if (res<0)
    {
      DEBUG("Write command failed with error code:");
      DEBUGLN(res);
      return;
    }
  }
}

void declare_new_node_to_server()
{
  // Here we want to send a NEW-NODE:... message to the server for nodes in ONLINE_NODE state
  // That is they are ready to work but not yet known from the server
  if (!online_nodes || !(client.connected()) || (millis()-last_server_send<SERVER_SEND_TOUT))
    return;
  node * current = nodes_head;
  while (current) {
    if (current->state==ONLINE_NODE)
      break;
    current = current->next;
  }
  if (current) {
    last_server_send = millis();
    // Found an ONLINE node, send its characteristics to the server
    client.print("NEW-NODE:");
    client.print(current->address);
    client.print(',');
    client.print(bytes_to_wire(current->input_sensors,9));
    client.print(',');
    client.print(bytes_to_wire(current->output_sensors,9));
    client.print(',');
    client.print(bytes_to_wire(current->turnouts,9));
    client.print(',');
    client.print(bytes_to_wire(current->sensors_states,current->nb_sensors/7+1));
    client.print(',');
    client.println(bytes_to_wire(current->turnouts_states,current->nb_turnouts/7+1));
    // Set state to CONFIRMED   
    current->state=CONFIRMED;
    // Update number of ONLINE nodes remaining
    online_nodes--;
  }
}

void loop() {
  static unsigned loops = 0;
  loops++; // Count loops to share time
  // Take care of blinking
  if (millis()-blink_time>blink_delay) {
    blink_time = millis();
    state=!state;
    digitalWrite(13,state?HIGH:LOW);   
  }
  
  // Make sure we are connected to server
  if (!client.connected()) {
    connect_to_server();
  }
  
  if (loops % 10==0) {
    // Every 10 loops
    // First node that must be tested first are the TO_DISCOVER type
    node * to_discover = first_timedout_node(TO_DISCOVER,0,nodes_head);
    byte address = 0;
    if (to_discover)
      address = to_discover->address;
    else if (last_discover_add<=62)
      next_discover_add();
    if (!address)
      address = last_discover_add;
    if (address && (address<=62)) {
      DEBUG("Trying to discover at address ");
      DEBUGLN(address);
      int err = store_eeprom(address);
      if (err>=0) {
        err = load_eeprom(address);
        if (err>=0) {
          err = show_tables_cmd(address);
          if (err==0) {
            node * new_node = find_node_from_add(address);
            if (new_node && (new_node->address==address)) {
              DEBUG("New node, address=");
              DEBUGLN(address);
            }
          }
        }
      }
    } else {
      // No discovery needed, try to wake dead nodes up
      // FIXME
      //DEBUG("should try to wake dead nodes up now!");
    }
  } else if (loops%10==5) {
    // Read states of all sensors/turnouts of nodes of type NEW_NODE
    // to bring them to ONLINE_NODE state
    node * new_node = first_timedout_node(NEW_NODE,0,nodes_head);
    if (new_node) {
      DEBUG("Bring node to ONLINE_NODE ");
      DEBUGLN(new_node->address);
      if (read_all(new_node)<0) {
        DEBUG("Error reading all sensors for address ");
        DEBUGLN(new_node->address);
      } else {
        if (read_all(new_node,true)<0) {
          DEBUG("Error reading all turnouts for address ");
          DEBUGLN(new_node->address);
        } else {
          new_node->state = ONLINE_NODE;
          // Indicate that there is one more ONLINE node
          online_nodes++;
        }
      }
    }
  }

  if (!client.connected())
    return;
  // Check network for commands from the server
  unsigned long proc_beg = millis();
  while (client.available() && (millis()-proc_beg<SERVER_READ_TOUT)) {
    char c = client.read();
    if (c=='\n') {
      // Full command received let process it
      process_server_cmd(server_cmd);
      server_cmd = "";
      break;
    } else server_cmd+=c;
  }
  declare_new_node_to_server();
}