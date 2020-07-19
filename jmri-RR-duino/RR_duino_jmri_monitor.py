import socket,time
import serial_bus
import json,sys
import RR_duino_messages as RR_duino

#constants
ANSWER_TIMEOUT=0.2  #time out for an answer (200ms)
DEAD_NODES_TIME=5  #time between trials to wake up dead nodes

class RR_duino_node:
    PING_TIMEOUT = 0.05   # 50ms between pings
    def __init__(self,address,version):
        self.address=address
        self.version=version
        self.sensors = {}  #dict of sensors config subaddress <-> (pin,type)
        self.turnouts = {} #dict of turnouts
        #config subaddress <->(servo_pin,straight pos,thrown pos [,relay pin 1, relay pin 2,pulse pin 1, pulse pin 2])
        self.last_ping = 0

    def get_sensors_config(self):
        #get dict of sensors and turnouts
        debug("Getting sensors config from node at ",self.address)
        answer = send_msg(RR_duino.RR_duino_message.build_load_from_eeprom(node.address))
        if answer is None or answer.get_error_code()!=0:
            debug("node at",node.address,"was unable to save its config to eeprom")                              
            return False
        answer = send_msg(RR_duino.RR_duino_message.build_save_to_eeprom(node.address))
        if answer is None or answer.get_error_code()!=0:
            debug("node at",node.address,"was unable to save its config to eeprom")                              
            return False
        command =RR_duino.RR_duino_message.build_show_cmd(self.address)
        self.sensors={}
        self.turnouts={}
        i=0
        error = False
        pending_answers = False
        answer = []
        done = False
        while not done:
            answer.append(send_msg(command))
            if answer[-1] is None or not answer[-1].is_answer_to_cmd(command.get_command()):
                debug("Bad answer when loading config from node at ",self.address)
                done = True
                error = True
            else:
                done = answer[-1].is_last_answer()
        debug("unable to get sensors config from node at",node.address)
        if not error:
            for m in answer:
                #concatenate dicts (python >=3.5)
                {**self.sensors,**(m.get_list_of_sensors_config())}
                debug("Sensors list=",self.sensors)
        if not pending_answer:  #reset ping time if no answer is pending, otherwise decrease by PING_TIMEOUT/5
            self.last_ping = time.time()
        else:
            self.last_ping -= RR_duino_node.PING_TIMEOUT/5
        return not error
    
    def show_config(self,fullID):
        res = json.dumps({"FULLID":fullID,"ADDRESS":self.address,"VERSION":self.version,"SENSORS":self.sensors,"TURNOUTS":self.turnouts})
        debug("show_config=",res)
        return res

def RR_duino_to_JMRI(msg):
    """
    Convert the binary RR_duino message to a text based to be sent to the jython script launched from JMRI
    Format: Prefix+address+":"+subaddress+","+0/1+";"
    Prefix :(default AS for sensors and AT for turnouts)
    address:address of the device (1-62)
    subadd :number to identify the sensor/turnout for this device

    returns None if an error occured, empty string if not to be sent to JMRI (eg an answer to a turnout cmd)
    or the converted message otherwise
    """
    if not msg.is_valid() or not msg.is_answer():
        debug("Invalid message or not an answer")
        return None
    if msg.is_write_cmd() or msg.is_config_cmd() or msg.is_special_config_cmd() or msg.is_all():
        if msg.is_all():
            debug("RR_duino message about all turnouts/sensors, not supported!")
        #do not send the answer to a write cmd or a config cmd
        return ""

    #OK so it is an answer to a read cmd

    #set prefix to sensor prefix as this is a read command (input sensors are treated as sensors in jmri
    #and turnout feedback also uses sensors)
    
    #position turnouts are translated as input sensors because of the way JMRI defines turnout feedback
    #the correspondence is as follows: if turnout is number 6 then the sensor will be 50+6=56
    #so AT12:6 feedback sensor will be AS12:56
    #CONSEQUENCE: only 49 turnouts can be used

    #output sensors are represented as turnouts with feedback in jmri so an output sensor feedback will be
    #translated as a sensor by adding 50 to its subaddress
    #so output sensor on device of address 12 and subaddress 15 will be translated as AS12:65
    #VERY IMPORTANT: this means that no output sensor can have the same subaddress a a turnout!
    #CONSEQUENCE: only 50 input sensors (subaddress from 1 to 50) are allowed

    node = node_from_address(msg.get_address(),online_nodes)
    prefix = config["sensor_prefix"]+str(msg.get_address())+":"
    if msg.is_list():
        #list of subbadd,value pairs
        result=""
        subadds_values = msg.get_list_of_values()
        for sub_val in subbadds_values:
            number = sub_value[0]
            if msg.on_turnout() or node.sensors[sub_val[0]][1]==RR_duino.RR_duino_message.OUTPUT_SENSOR:
                #make sure to translate the sensor number for turnouts or output sensors
                number += 50
            result += prefix+str(number)+","+str(sub_value[1])+";"
    else:
        subadd,value = msg.get_value()
        if msg.on_turnout() or node.sensors[subadd][1]==RR_duino.RR_duino_message.OUTPUT_SENSOR:
            #make sure to translate the sensor number for turnouts or output sensors
            subadd += 50
        return prefix+str(subadd)+","+str(value)+";"

def JMRI_to_RR_duino(message):
    """
    Convert the text based to a binary RR_duino message to be sent to the nodes
    Format: Prefix+address+":"+subaddress+","+0/1+";"
    Prefix :(default AS for sensors and AT for turnouts)
    address:address of the device (1-62)
    subadd :number to identify the sensor/turnout for this device

    returns None if an error occured or the converted message otherwise
    """

    message = message.rstrip(";")
    if  message.startswith(config["sensor_prefix"]):
        for_sensor = True
    elif message.startswith(config["turnout_prefix"]):
        for_sensor = False
    else:
        debug("Malformed message0:",message,"from jmri!")
        return None
    
    first,sep,end = message.partition(":")
    if sep=="":
        debug("Malformed message1:",message,"from jmri!")
        return None

    print("first=",first[len(config["turnout_prefix"]):])
    try:
        address = int(first[len(config["turnout_prefix"]):])
    except:
        debug("Malformed message2:",message,"from jmri!")
        return None
    node = node_from_address(address,online_nodes)
    if node is None:
        debug("Unknown address:",address,"from jmri!")
        return None

    first,sep,end = end.partition(",")
    if sep=="" and not for_sensor:
        debug("No value given for turnout!")
        return
    try:
        subadd = int(first)
    except:
        debug("Malformed message3:",message,"from jmri!")
        return None
    if not for_sensor:
        try:
            value = int(end)
        except:
            debug("Malformed message4:",message,"from jmri!")
            return None
        if value != 0:
            value = 1
    else:
        value = 0

    #build RR_duino binary message
    result = RR_duino.RR_duino_message.build_rw_cmd_header(address,for_sensor,for_sensor,False)
    result.extend_payload(RR_duino.RR_duino_message.encode_subadd_value((subadd,value)))
    return result

def decode_messages():
    global rcv_messages,rcv_RR_messages

    sep = ";"
    while sep==";":
        first,sep,end = rcv_messages.partition(";")
        if sep!="":
            #FIXME: need to convert from text messages received from the jmri script (easier to debug in jmri)
            msg = JMRI_to_RR_duino(first)
            if msg is not None:
                rcv_RR_messages.append(msg)
            rcv_messages = end

def node_from_address(add,nodes_list):
    for node in nodes_list:
        if node.address == add:
            return node
    return None

def debug(*args):
    print(*args)
    
def discover_next_node():
    #Check if we dont hog the bus bandwidth too much
    if len(online_nodes)>0:
        #we must wait between discovers, more so if we already have more nodes
        #to give them time to come online
        if time.time()-last_discover<len(online_nodes)/2*ANSWER_TIMEOUT:
            #too soon just return
            return
    last_discover = time.time()
    #First find the next undiscovered address
    done = False
    while not done:
        discover_address+=1
        done = True
        for n in online_nodes:
            if n.address==discover_address:
                discover_address+=1
                done=False
                break
    if discover_address>62:
        #all addresses have been tried already (max is 62)
        return
    #build a "version cmd" message to see if someone is responding
    msg = RR_duino.RR_duino_message.build_version_cmd(node_to_ping.address)
    ser.send(msg.raw_message)
    waiting_answer_from = None
    waiting_answer_from_add = discover_address
    answer_clock = time.time()    

def process():
    global rcv_RR_messages,ser,waiting_answer_from,answer_clock,last_dead_nodes_ping

    def find_oldest_ping(nodes_list):
        older_ping = time.time()
        node_to_ping = None
        for node in nodes_list:
            if node.last_ping < older_ping:
                older_ping = node.last_ping
                if older_ping < time.time()-RR_duino_node.PING_TIMEOUT:
                    #debug("times:",time.time(),older_ping, node.address)
                    node_to_ping = node
        return node_to_ping
    
    if ser.sending():
        #if we are already sending
        #not much we can do, let's wait for IO to proceed
        return
    
    if waiting_answer_from is not None or waiting_answer_from_add!=0:   #we are waiting for an answer
        if time.time()>answer_clock+ANSWER_TIMEOUT:
            if waiting_answer_from is None:
                #we were waiting for an answer because we were trying to find a new node
                #no answer, just give up
                debug("auto-discover, node at",waiting_answer_from_add," not found")
                waiting_answer_from_add = 0
            elif waiting_answer_from in online_nodes:
                #timeout for an answer->node is down
                debug("node of address", waiting_answer_from.address,"is down")
                debug(answer_clock,time.time(),waiting_answer_from.address)
                dead_nodes.append(waiting_answer_from)
                online_nodes.remove(waiting_answer_from)
            waiting_answer_from = None
            message_to_send = b""
        else:
            if ser.available():     # we are receiving an answer
                message_to_send += ser.read()
                #debug("message_to_send=",message_to_send)

            if not RR_duino.RR_duino_message.is_complete_message(message_to_send):
                return
            #answer complete, first check if it is from a dead node or a new node that we just discovered
            msg = RR_duino.RR_duino_message(message_to_send)
            if waiting_answer_from is None: #new node just discovered
                #check correct protocol
                if not msg.is_answer_to_version_cmd() or msg.get_address()!=waiting_answer_from_add:
                    debug("Discovered node of address",waiting_answer_from_add,"gave an unexpected answer (should have replied to a version command) or the responding node does not have the correct address:",msg.get_address())
                else:
                    #new node, build it using the version we got from it
                    discovered_nodes.append(RR_duino_node(waiting_answer_from_add,msg.get_version()))
                    #now get its config
                    msg = RR_duino.RR_duino_message.build
                    ser.send(msg)
                #unset address, we are not waiting anymore
                waiting_answer_from_add = 0
                
            elif waiting_answer_from in dead_nodes:
                debug("node at",waiting_answer_from.address,"has woken up")
                online_nodes.append(waiting_answer_from)
                dead_nodes.remove(node_to_ping)
            elif waiting_answer_from in online_nodes:
                debug("received from serial and sending it to the server:",(msg.to_wire_message()+";").encode('utf-8'))
                if jmri_sock is None:
                    debug("jmri is not yet connected!")
                else:
                    #convert RR_duino binary message to text based message for jmri (easier to debug)
                    message = RR_duino_to_JMRI(msg)
                    if message is not None and len(message)>0:
                        jmri_sock.send(message.encode('utf-8'))
                    if message is not None and msg.async_events_pending():
                        #pending answers so decrease last ping time to boost its priority
                        waiting_answer_from.last_ping -= RR_duino_node.PING_TIMEOUT / 5    
            #reset
            message_to_send=b""
            waiting_answer_from = None

    #not waiting for an answer
    #if auto-discover is set, try to find a new node
    if config["auto_discover"] and discover_address<=62:
        discover_next_node()
        return
    #let's send a new command if there is any
    if rcv_RR_messages:
        msg = rcv_RR_messages.pop(0)
        node = node_from_address(msg.get_address(),online_nodes)
        if node is None or node not in online_nodes:
            #ignore message to disappeared nodes
            return
        debug("Processing message from server", msg.to_wire_message())
        ser.send(msg.raw_message)
        waiting_answer_from = node
        answer_clock = time.time()
    else:
        #no ongoing I/O on the bus check the node with older ping
        node_to_ping = find_oldest_ping(online_nodes)
        if node_to_ping is not None:
            debug("pinging node at",node.address)
            node_to_ping.last_ping = time.time()
            msg = RR_duino.RR_duino_message.build_async_cmd(node_to_ping.address)
            ser.send(msg.raw_message)
            waiting_answer_from = node_to_ping
            answer_clock = time.time()
        #no node to ping, try to wake dead nodes up:
        elif time.time()>last_dead_nodes_ping+DEAD_NODES_TIME:
            #try to wake up a "dead" node
            debug("trying to wake dead nodes up")
            node_to_wake = find_oldest_ping(dead_nodes)
            if node_to_ping is not None:
                node_to_wake.last_ping = time.time()
                msg = RR_duino.RR_duino_message.build_async_cmd(node_to_wake.address)
                ser.send(msg.raw_message)
                waiting_answer_from = node_to_wake
            last_dead_nodes_ping = time.time()

def load_config(filename):
    #Load config file (json formatted dict config, see below)

    with open(filename) as cfg_file:
        config = json.load(cfg_file)
    if "serial_port" not in config:
        debug("No serial port indicated!")
        return None
    #plug reasonable default values for secondary parameters
    if "serial_speed" not in config:
        config["serial_speed"]=19200
    #network port to listen to
    if "network_port" not in config:
        config["network_port"]=50010
    #if auto_discover is not set, set it to True
    if "auto_discover" not in config:
        config["auto_discover"]=True
    #list of nodes addresses (can be empty, will trigger auto-discover
    if "nodes_addresses" not in config:
        config["nodes_addresses"]=[]
    #prefix for sensors/turnouts in jmri
    if "sensors_prefix" not in config:
        config["sensor_prefix"]="AS"
    if "turnouts_prefix" not in config:
        config["turnout_prefix"]="AT"

    return config

def poll_net():
    global rcv_messages,jmri_sock
    #first check network connections
    ready_to_read,ready_to_write,in_error = select.select(to_read,[],[],timeout)
    if serversocket in ready_to_read:
        jmri_sock,addr = self.serversocket.accept()
        address = (str(addr).split("'"))[1]
        debug("Got a connection from", address)
        ready_to_read.remove(serversocket)
    #check if we got something from jmri (through net)
    if jmri_sock is not None and jmri_sock in ready_to_read:
        #we only read from the first socket (which should be JMRI)
        try:
            m = read_to_read[0].recv(200).decode('utf-8')
        except socket.error:
            debug("recv error")
            #debug(len(m)," => ",m)
            if not m:
                #ready to read and empty msg means deconnection
                print("JMRI has deconnected")
            else:
                rcv_messages+=m
                decode_messages()

def send_msg(msg):
    #send a message to the serial port and wait for the answer (or timeout)
    #return answer or None if timed out
    #this should be called when no message is processed on the serial bus
    global ser

    ser.send(msg.raw_message)
    answer = bytearray()
    begin = time.time()
    complete = False
    #wait for answer completion (or time out)
    while not complete and time.time()<begin+ANSWER_TIMEOUT: #not complete yet
        ser.process_IO()
        r = ser.read()
        if r is not None:
            begin = time.time()
            answer.extend(r)
            complete = RR_duino.RR_duino_message.is_complete_message(answer)
        #check net connection also
        poll_net()
        
    #check time out and answer begins by START and is the answer to the command we have sent
    if time.time()<begin+ANSWER_TIMEOUT:
        answer_msg =  RR_duino.RR_duino_message(answer)
        if answer_msg.is_valid() and answer_msg.is_answer_to_cmd(msg.raw_message[1]):
            return answer_msg
        else:
            debug("Broken protocol: command was:", msg.to_wire_message()," answer : ",answer_msg.to_wire_message())
    debug("Timed out!")
    return None
    
def load_nodes():
    global online_nodes
    debug("loading RR_duino nodes with addresse",config["nodes_addresses"])

    #try all addresses and ask each responding node to load its version
    for address in config["nodes_addresses"]:
        answer = send_msg(RR_duino.RR_duino_message.build_version_cmd(address))
        if answer is not None and answer.get_error_code()==0:
            #add the node to the online list
            new_node = RR_duino_node(address,answer.get_version())
            discovered_nodes.append(new_node)
        else:
            debug("node at",address,"did not respond")

    #get online node to upload its config (load from eeprom and set save to eeprom flag)
    for node in discovered_nodes:
        #prepare to discard node in case of any error
        if not node.get_sensors_config():
            debug("node at",node.address,"was unable to load its config from eeprom")
            onlines_nodes.append(node)
        else:
            debug("node at",node.address,"is up and running")
    #remove the node that were not able to go online correctly
    discovered_nodes = []


if len(sys.argv)>=2:
    config = load_config(sys.argv[1])
else:
    config = load_config("RR_duino_jmri_monitor.cfg")

if config is None:
    quit()

"""
Tests
n1 = RR_duino_node(5,1)
online_nodes = [n1,RR_duino_node(2,1),RR_duino_node(3,1)]
n1.sensors[2]=(1,RR_duino.RR_duino_message.INPUT_SENSOR)
n1.sensors[7]=(1,RR_duino.RR_duino_message.OUTPUT_SENSOR)
while True:
    s = input("command:")
    if s=="STOP":
        break
    msg = JMRI_to_RR_duino(s)
    if msg is not None:
        print(msg.to_wire_message())
msg = RR_duino.RR_duino_message.build_simple_rw_cmd(5,2,read=True)
msg.raw_message[1] &= ~(1 << RR_duino.RR_duino_message.CMD_ANSW_BIT)
print(msg.to_wire_message(),"<-->",RR_duino_to_JMRI(msg))
msg = RR_duino.RR_duino_message.build_simple_rw_cmd(5,7,read=True)
msg.raw_message[1] &= ~(1 << RR_duino.RR_duino_message.CMD_ANSW_BIT)
print(msg.to_wire_message(),"<-->",RR_duino_to_JMRI(msg))
msg = RR_duino.RR_duino_message.build_simple_rw_cmd(5,8,True,False,1)
msg.raw_message[1] &= ~(1 << RR_duino.RR_duino_message.CMD_ANSW_BIT)
print(msg.to_wire_message(),"<-->",RR_duino_to_JMRI(msg))
msg = RR_duino.RR_duino_message.build_simple_rw_cmd(5,8,True,False,0)
msg.raw_message[1] &= ~(1 << RR_duino.RR_duino_message.CMD_ANSW_BIT)
print(msg.to_wire_message(),"<-->",RR_duino_to_JMRI(msg))
quit()
"""
#connection to the serial bus
ser = serial_bus.serial_bus(config["serial_port"],config["serial_speed"])

debug("RR_duino_net_serial started on serial port",config["serial_port"],"at",config["serial_speed"])
connected = False
while not connected:
    try:
        ser.start()
        connected=True
    except serial.serialutil.SerialException:
        time.sleep(1)
        debug("Waiting to connect to serial port",config["serial_port"])
        pass
debug("Connected to serial port",config["serial_port"])

rcv_messages=""  #last received messages ready to be cut and decoded
rcv_RR_messages = []  #decoded RR_duino messages received from the gateway, waiting to be sent
message_to_send=b""   #last incomplete message from the serial port

#variables used to know if a node should respond and which one
waiting_answer_from = None #this will be None if we are trying to discover a new node
wating_answer_from_add = 0 #corresponding address, useful when you are trying to discover a new node
answer_clock = 0

time.sleep(1) #time for arduino serial port to settle down

#server connection
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_add = "127.0.0.1" #FIXME
serversocket.bind((server_add, config["network_port"]))
debug("RR_duino_monitor listening on ",server_add," at port ",config["network_port"])
serversocket.listen(5)

#load nodes from files and bring them online
load_nodes()
#list of dead nodes (these were online and died, we keep them and try to see if they wake up)
dead_nodes=[]
last_dead_nodes_ping = time.time()
#list of online nodes
online_nodes = []
#list of newly discovered nodes but not yet online (need to get their config first)
discovered_nodes = []
#last address for which we tried to discover a new node
discover_address=0
#last time we tried to discover a new node
last_discover = 0
#sockets to read
to_read=[serversocket]
#socket to jmri
jmri_sock = None

while True:
    #network
    poll_net()
    #process serial I/O
    ser.process_IO()
    process()
        
ser.close()
