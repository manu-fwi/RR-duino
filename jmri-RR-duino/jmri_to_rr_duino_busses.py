import socket,select,time
import json,sys

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

class RRduino_bus:
    def __init__(self,number,sock):
        self.number = number
        self.sock = sock
        self.last_message = ""
        self.msgs_list = []

    def decode_last_message(self):
        new = False
        while True:
            begin,sep,end = self.last_message.partition("\n")
            if sep == "":
                return new
            self.msgs_list.append(begin.rstrip())
            self.last_message = end.lstrip()
            new = True

    def process(self):
        global jmri_sock
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
                pos_bytes = [int(s,16) for s in parts[1].lstrip().split(" ")]
                input_sensors_sub = [pos+1 for pos in pos_bits_set(pos_bytes)]
                pos_bytes = [int(s,16) for s in parts[2].lstrip().split(" ")]
                output_sensors_sub = [pos+1 for pos in pos_bits_set(pos_bytes)]
                pos_bytes=[int(s,16) for s in parts[3].lstrip().split(" ")]
                turnouts_sub = [pos+1 for pos in pos_bits_set(pos_bytes)]
                sensors_states = [int(s,16) for s in parts[4].lstrip().split(" ")]
                turnouts_states = [int(s,16) for s in parts[5].lstrip().split(" ")]
                #merge input and output sensors subaddresses in one list
                all_sensors_sub = input_sensors_sub[:]
                all_sensors_sub.extend(output_sensors_sub)
                all_sensors_sub.sort()
                index = 0
                #generate a message for each sensor to signal its initial state
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
                        index+=1
                    if index>=(len(input_sensors_sub)+len(output_sensors_sub)):
                        #we have checked state for all sensors
                        break                
                debug(input_sensors_sub)
                debug(output_sensors_sub)
                debug(turnouts_sub)
                debug(sensors_states)
                debug(turnouts_states)
            else:
                #should be a message to send to jmri so
                if jmri_sock is None:
                    debug("Received",msg,"but jmri is not connected yet!")
                else:
                    #add bus number to it and send it to jmri
                    to_send=msg[:4]+str(self.number)+":"+msg[4:]+"\r\n"
                    jmri_sock.send(to_send.encode('utf-8'))
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

def process_jmri():
    """
    send messages from jmri to the correct RRduino bus
    """
    
    global jmri_last_message,jmri_msgs_list

    sep = "\n"
    while sep!="":
        first,sep,end = jmri_last_message.partition("\n")
        if sep!="":
            jmri_msgs_list.append(first.rstrip())
            jmri_last_message = end.lstrip()

    for msg in jmri_msgs_list:
        bus_n = get_bus_number(msg)
        if bus_n != -1:
            bus = get_bus_by_number(bus_n)
            if bus is not None:
                bus.send(msg)
    jmri_msgs_list=[]

def debug(*args):
    print(*args)
    
def load_config(filename):
    #Load config file (json formatted dict config, see below)

    with open(filename) as cfg_file:
        config = json.load(cfg_file)
    #network ports to listen to
    if "jmri_port" not in config:
        config["jmri_port"]=50010
    if "rrduino_busses_port" not in config:
        config["rrduino_busses_port"]=50011
    
    return config

if len(sys.argv)>=2:
    config = load_config(sys.argv[1])
else:
    config = load_config("jmri_to_rr_duino_busses.cfg")

if config is None:
    quit()

jmri_last_message=""  #last received messages ready to be cut and decoded
jmri_msgs_list = []   #list of msgs from jmri

#server connection
jmri_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_add = "192.168.0.22" #FIXME
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

jmri_sock = None
jmri_add = None

while True:
    #first check network connections JMRI first
    ready_to_read,ready_to_write,in_error = select.select(to_read_jmri,[],[],timeout)
    if jmri_server_sock in ready_to_read:
        jmri_sock,addr = jmri_server_sock.accept()
        jmri_add  = (str(addr).split("'"))[1]
        debug("Got a JMRI connection from", jmri_add)
        ready_to_read.remove(jmri_server_sock)
        to_read_jmri.append(jmri_sock)
    #check if we got something from jmri (through net)
    if jmri_sock in ready_to_read:
        m=""
        try:
            m = jmri_sock.recv(200).decode('utf-8') #FIXME: only one jmri client is allowed for now
        except socket.error:
            debug("recv error")
            #debug(len(m)," => ",m)
        if not m:
            #ready to read and empty msg means deconnection
            print("JMRI Client has deconnected")
            to_read_jmri.remove(jmri_sock)
        else:
            jmri_last_message+=m

    process_jmri()

    #next connections from the RRduino busses
    ready_to_read,ready_to_write,in_error = select.select(to_read_busses,[],[],timeout)
    if busses_server_sock in ready_to_read:
        bus_sock,addr = busses_server_sock.accept()
        jmri_add  = (str(addr).split("'"))[1]
        debug("Got a BUS connection from", jmri_add)
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
