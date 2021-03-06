import socket,select,time
import json,sys

#config file format
#a dictionnary bus number (as string)<-> boolean, that is {"1":True,...}
#that tells if the bus controller can do auto discovery
#a dictionnary bus number (as string) <-> list of addresses, that is {"1":[1,25,78,3],...}
#where the addresses are the one that should be discovered first (and only them if autodiscover is not allowed
#this is to speed up the process of discovery
#constants
timeout=0.01

def pos_bits_set(bits):
    """
    returns a list of the positions of each bit set in the bits array (it is a byte array)
    Only the 7 LSB are taken into account, MSB is always 0 and not counted in
    """

    positions = []
    bit_pos=0
    for b in bits:
        for pos in range(7):
            if (b & (1 << pos))!=0:
                positions.append(bit_pos)
            bit_pos+=1
    return positions

class JMRI_connection:
    def __init__(self):
        self.sock = None
        #lists of messages to be sent to jmri (jmri may connect later)
        self.important_msgs = []
        self.states_msgs = []
        #last received messages ready to be cut and decoded
        self.last_message=""
        #list of msgs from jmri
        self.msgs_list = []

    def read_net(self):
        m=""
        try:
            m = self.sock.recv(200).decode('utf-8') #FIXME: only one jmri client is allowed for now
        except socket.error:
            debug("recv error")
            #debug(len(m)," => ",m)
        if not m:
            #ready to read and empty msg means deconnection
            print("JMRI Client has deconnected")
            to_read_jmri.remove(self.sock)
        else:
            self.last_message+=m
            debug("received from jmri:",m)
            
    def flush_msgs(self):
         #socket is ready so send everything waiting, beginning by important msgs

        for m in self.important_msgs:
            self.sock.send(m.encode('utf-8'))
            debug("flush_msgs",m)
        self.important_msgs = []
        for m in self.states_msgs:
            self.sock.send(m.encode('utf-8'))
            debug("flush_msgs",m)
        self.states_msgs = []
        
    def send_important_msg(self,msg):
        self.important_msgs.append(msg)
        if self.sock is not None:
            self.flush_msgs()
            
    def send_states_msg(self,msg):
        self.states_msgs.append(msg)
        if self.sock is not None:
            self.flush_msgs()

    def process(self):
        """
        send messages from jmri to the correct RRduino bus
        """
        if self.sock is None:
            return
        else:
            self.flush_msgs()
        if self.last_message!="":
            sep = "\r\n"
            while sep!="":
                first,sep,end = self.last_message.partition(sep)
                #debug(first,"/",sep,"/",end)
                if sep!="":
                    self.msgs_list.append(first.rstrip())
                    self.last_message = end.lstrip()

        for msg in self.msgs_list:
            debug("processing message from jmri:",msg)
            bus_n = get_bus_number(msg)
            if bus_n != -1:
                bus = get_bus_by_number(bus_n)
                if bus is not None:
                    bus.send(msg)
        self.msgs_list=[]
    
class RRduino_bus:
    def __init__(self,number,sock):
        self.number = number
        self.sock = sock
        self.last_message = ""
        self.msgs_list = []
        self.auto_discover = True
        self.discover_adds=None
        self.config_loaded = False

    def decode_last_message(self):
        new = False
        while True:
            begin,sep,end = self.last_message.partition("\r\n")
            if sep == "":
                return new
            self.msgs_list.append(begin.rstrip())
            self.last_message = end.lstrip()
            new = True

    def declare_jmri_objects(self,address,subaddresses,offset,is_turnout):
        if is_turnout:
            msg="NEW-TURNOUTS:"
        else:
            msg="NEW-SENSORS:"
        for s in subaddresses:
            msg+=str(self.number)+":"+str(address)+":"+str(s+offset)+" "
        msg+="\r\n"
        
        jmri.send_important_msg(msg)

    def get_subaddresses(self,hex_list):
        pos_bytes = [int(s,16) for s in hex_list.lstrip().split(" ")]
        return [pos+1 for pos in pos_bits_set(pos_bytes)]

    def get_config(self):
        if not self.config_loaded:
            if str(self.number) in config["busses_config"]:
                self.auto_discover = config["busses_config"][str(self.number)]
            if str(self.number) in config["nodes_addresses"]:
                self.discover_adds = config["nodes_addresses"][str(self.number)]

        res = "CONFIG "
        if not self.auto_discover:
            res+="NO-"
        res+="AUTO-DISCOVER"
        if self.discover_adds is not None:
            res+=" ADDRESSES:"
            first = True
            for add in self.discover_adds:
                if add>0 and add<62:
                    if first:
                        first = False
                    else:
                        res+=","
                    res+=str(add)
        return res+"\r\n"
                
    def process(self):
        for msg in self.msgs_list:
            if msg.startswith("RRDUINO-BUS"):
                #new bus
                bus_n = -1
                try:
                    bus_n = int(msg[len("RRDUINO-BUS"):].lstrip())
                except:
                    debug("Error in msg",msg)
                if bus_n>=0:
                    self.number = bus_n
                    #send the config to the bus controller
                    self.sock.send(self.get_config().encode('utf-8'))
                    debug("sending",self.get_config(),"to bus",bus_n)
            elif msg.startswith("NEW-NODE"):
                debug("New node from",self.number,msg)
                #New node came up: send all states (for input sensors but also for turnouts feedbacks)
                parts = msg.split(",")
                debug(parts)
                if len(parts)!=6:
                    debug("Invalid NEW-NODE: message structure!")
                #get address
                addstr = (parts[0][parts[0].find(":")+1:]).lstrip()
                try:
                    address = int(addstr)
                except:
                    debug("Address is invalid in NEW-NODE message!")
                    address = -1
                if address < 0:
                    return
                input_sensors_sub = self.get_subaddresses(parts[1])
                output_sensors_sub =  self.get_subaddresses(parts[2])
                turnouts_sub =  self.get_subaddresses(parts[3])
                sensors_states = [int(s,16) for s in parts[4].lstrip().split(" ")]
                turnouts_states = [int(s,16) for s in parts[5].lstrip().split(" ")]
                
                #declare input sensors to JMRI script as sensors in JMRI, all turnouts MUST be declared first
                #as some sensors are actually only feedback for them, the jython script will try to associate
                #them to the corresponding turnouts when the sensor is created.
                #declare output sensors to JMRI script as turnouts in JMRI (subaddress+100 as number)
                self.declare_jmri_objects(address,output_sensors_sub,100,True)
                #declare turnouts to JMRI script as turnouts in JMRI
                self.declare_jmri_objects(address,turnouts_sub,0,True)
                self.declare_jmri_objects(address,input_sensors_sub,0,False)
                #declare output sensors feedback to JMRI script as sensors in JMRI (subaddress+200 as number)
                self.declare_jmri_objects(address,output_sensors_sub,200,False)
                #declare turnout sensors feedback to JMRI script as sensors in JMRI (subaddress+100 as number)
                self.declare_jmri_objects(address,turnouts_sub,100,False)
                
                #merge input and output sensors subaddresses in one list
                all_sensors_sub = input_sensors_sub[:]
                all_sensors_sub.extend(output_sensors_sub)
                all_sensors_sub.sort()
                index = 0
                #generate a message for each sensor to signal its initial state
                debug("----------------------------Initial states----------------------")
                for state in sensors_states:
                    for bit_pos in range(7):
                        if index>=(len(input_sensors_sub)+len(output_sensors_sub)):
                            #we have checked state for all sensors
                            break
                        #get bit value
                        value = (state & (1<<bit_pos))
                        if value!=0:
                            #normalize value
                            value=1
                        subaddress = all_sensors_sub[index]
                        if subaddress in output_sensors_sub:
                            #it is an output sensor for RRduino, translate to a turnout for jmri (number=subaddress+200)
                            subaddress+=200
                        self.msgs_list.append("ISRS"+str(address)+":"+str(subaddress)+","+str(value))
                        debug("ISRS"+str(address)+":"+str(subaddress)+","+str(value))
                        index+=1
                    if index>=(len(input_sensors_sub)+len(output_sensors_sub)):
                        #we have checked state for all sensors
                        break
                #same for turnouts
                index=0
                for state in turnouts_states:
                    for bit_pos in range(7):
                        if index>=len(turnouts_sub):
                            #we have checked state for all turnouts
                            break
                        #get bit value
                        value = (state & (1<<bit_pos))
                        if value!=0:
                            #normalize value
                            value=1
                        subaddress = turnouts_sub[index]
                        self.msgs_list.append("ISRS"+str(address)+":"+str(subaddress+100)+","+str(value))
                        debug("ISRS"+str(address)+":"+str(subaddress+100)+","+str(value))
                        index+=1

                debug("------------------------End of initial states--------------------")
                debug(input_sensors_sub)
                debug(output_sensors_sub)
                debug(turnouts_sub)
                debug(sensors_states)
                debug(turnouts_states)
            else:
                #add bus number to it and send it to jmri
                to_send=msg[:4]+str(self.number)+":"+msg[4:]+"\r\n"
                jmri.send_states_msg(to_send)
                debug("Sending",to_send,"to jmri")
        self.msgs_list = []

    def send(self,msg):
        #strip the bus number out of the msg
        msg = msg[:4] + msg[msg.find(":")+1:]
        self.sock.send((msg+"\r\n").encode('utf-8'))
        debug("Sending",msg,"to bus number",self.number)
        

def get_bus_by_number(bus_nb):
    for sock in busses:
        if busses[sock].number == bus_nb:
            return busses[sock]
    return None

def get_bus_number(msg):
    index = msg.find(":")
    if index==-1:
        debug("No bus number in msg from jmri!",msg)
        return -1
    try:
        bus_int = int(msg[4:index])
    except:
        debug("Bad bus number in msg from jmri!",msg)
        return -1
    return bus_int

def debug(*args):
    print(*args)

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def load_config(filename):
    #Load config file (json formatted dict config, see below)

    with open(filename) as cfg_file:
        config = json.load(cfg_file)
    #network ports to listen to
    if "jmri_port" not in config:
        config["jmri_port"]=50010
    if "rrduino_busses_port" not in config:
        config["rrduino_busses_port"]=50011
    if "listening_ip" not in config:
        config["listening_ip"]=get_ip()

    #nodes addresses by bus (this is to speed up nodes discovery)
    if "nodes_addresses" not in config:
        config["nodes_addresses"]={}

    if "busses_config" not in config:
        config["busses_config"]={}
    
    return config

if len(sys.argv)>=2:
    config = load_config(sys.argv[1])
else:
    config = load_config("jmri_to_rr_duino_busses.cfg")

if config is None:
    quit()

#server connection
jmri_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_add = config["listening_ip"]
jmri_server_sock.bind((server_add, config["jmri_port"]))
busses_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
busses_server_sock.bind((server_add, config["rrduino_busses_port"]))
debug("RR_duino_monitor listening on ",server_add," at ports ",config["jmri_port"],"and",config["rrduino_busses_port"])
jmri_server_sock.listen(5)
busses_server_sock.listen(5)

#list of existing busses
busses = {}

#sockets to read
to_read_jmri=[jmri_server_sock]
to_read_busses=[busses_server_sock]

jmri = JMRI_connection()

while True:
    #first check network connections JMRI first
    ready_to_read,ready_to_write,in_error = select.select(to_read_jmri,[],[],timeout)
    if jmri_server_sock in ready_to_read:
        jmri.sock,addr = jmri_server_sock.accept()
        jmri_add  = (str(addr).split("'"))[1]
        debug("Got a JMRI connection from", jmri_add)
        ready_to_read.remove(jmri_server_sock)
        to_read_jmri.append(jmri.sock)
    #check if we got something from jmri (through net)
    if jmri.sock in ready_to_read:
        jmri.read_net()

    jmri.process()

    #next connections from the RRduino busses
    ready_to_read,ready_to_write,in_error = select.select(to_read_busses,[],[],timeout)
    if busses_server_sock in ready_to_read:
        bus_sock,addr = busses_server_sock.accept()
        add  = (str(addr).split("'"))[1]
        debug("Got a BUS connection from", add)
        ready_to_read.remove(busses_server_sock)
        to_read_busses.append(bus_sock)
        #add bus the busses dictionnary, to be filled correctly later on
        busses[bus_sock]=RRduino_bus(None,bus_sock)
        
    #check if we got something from busses
    for sock in ready_to_read:
        m=""
        try:
            m = sock.recv(200).decode('utf-8') #FIXME
        except socket.error:
            debug("recv error")
            #debug(len(m)," => ",m)
        if not m:
            #ready to read and empty msg means deconnection
            print("BUS Client",busses[sock].number,"has deconnected")
            del busses[sock]
            to_read_busses.remove(sock)
        else:
            busses[sock].last_message+=m
            if busses[sock].decode_last_message():
                busses[sock].process()
