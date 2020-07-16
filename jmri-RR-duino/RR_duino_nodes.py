#from openlcb_cmri_cfg import hex_int
#from openlcb_protocol import *
#from openlcb_debug import *
#import openlcb_config,openlcb_nodes,collections

class RR_duino_message:
    START=0xFF
    #command byte special bits positions
    CMD_ANSW_BIT = 0
    CMD_LAST_ANSW_BIT=1
    CMD_ASYNC_BIT=2
    CMD_SPECIAL_CONFIG_BIT=3
    CMD_RW_BIT=5
    CMD_SENSOR_TURNOUT_BIT=4
    CMD_CONFIG_DEL_BIT=5
    CMD_ALL_BIT=6
    CMD_CONFIG_BIT = 7

    CMD_SPECIAL_CONFIG_CODE_POS=4
    CMD_SPECIAL_CONFIG_CODE_MASK=7

    CMD_VERSION = 0
    CMD_SET_ADDRESS = 1
    CMD_STORE_EEPROM = 2
    CMD_LOAD_EEPROM = 3
    CMD_SHOW_SENSORS = 4
    CMD_SHOW_TURNOUTS = 5
    CMD_TURNOUT_FINE_TUNE=6
    CMD_CLEAR_EEPROM = 7
    
    #address byte special bits positions
    ADD_LIST_BIT=6
    
    #subaddress byte special bits positions
    SUBADD_VALUE_BIT = SUBADD_SENSOR_IO_BIT=SUBADD_TURNOUT_RELAY_PINS_BIT=6
    SUBADD_LAST_IN_LIST_BIT=7
    
    #subaddress byte special bits positions
    PIN_PULSE_BIT = 7
    PIN_PULLUP_BIT = 7

    #other constants
    INPUT_SENSOR = 0
    INPUT_SENSOR_PULLUP = 1
    OUTPUT_SENSOR=2

    
    def __init__(self,raw_message=None):  # raw_message must be a bytearray
        self.raw_message = raw_message
        
    def set_header(self,command,address):
        self.raw_message = bytearray((RR_duino_message.START,command,address))

    def extend_payload(self,array_b):
        self.raw_message.extend(array_b)

    def get_address(self):
        return self.raw_message[2] & 0x3F

    def get_command(self):
        return self.raw_message[1]
    
    def get_version(self):
        return self.raw_message[2]

    def is_valid(self):
        #crude test about correctness: only check the start byte for now
        return self.raw_message[0]==RR_duino_message.START
    
    def is_answer(self):
        #return True if this message is an answer from the device
        return (self.raw_message[1] & (1 << RR_duino_message.CMD_ANSW_BIT))==0

    def is_answer_to_cmd(self,cmd):
        #return True if this message is an answer to the command "cmd"
        #fixme
        if not self.is_answer():
            return False
        #check if async bit was set in command
        if cmd & (1 << RR_duino_message.CMD_ASYNC_BIT) != 0:
            return (self.raw_message[1] & (1 << RR_duino_message.CMD_ASYNC_BIT)) !=0
        #check base command code
        if (self.raw_message[1] & 0b11111000) != (cmd & 0b11111000):
            return False
        return True

    def is_answer_to_version_cmd(self):
        return self.is_answer_to_cmd(0b10001001)  #version command value
    
    def is_last_answer(self):
        #return True if this message is the last answer (used mainly by show commands/async events reporting)
        return (self.raw_message[1] & (1 << RR_duino_message.CMD_LAST_ANSW_BIT))==0

    def is_read_cmd(self):
        return not self.is_config_cmd() and (self.raw_message[1] & (1 << RR_duino_message.CMD_RW_BIT))==0

    def is_write_cmd(self):
        return not self.is_config_cmd() and (self.raw_message[1] & (1 << RR_duino_message.CMD_RW_BIT))!=0

    def is_config_cmd(self):
        #returns True if this is a config command (might be a special config command)
        return (self.raw_message[1] & (1 << RR_duino_message.CMD_CONFIG_BIT))!=0

    def get_special_config(self):
        #returns the special config code (cf top of the class)
        #use it only if the command is a special config command
        return (self.raw_message[1] >> RR_duino_message.CMD_SPECIAL_CONFIG_CODE_POS ) & RR_duino_message.CMD_SPECIAL_CONFIG_CODE_MASK
    
    def is_special_config_cmd(self):
        #returns True if this is a special config command
        return self.is_config_cmd() and (self.raw_message[1] & (1 << RR_duino_message.CMD_SPECIAL_CONFIG_BIT))!=0

    def on_turnout(self):
        #returns True if this command is about sensors
        return (self.raw_message[1] & (1 << RR_duino_message.CMD_SENSOR_TURNOUT_BIT))!=0
    
    def async_events_pending(self):
        #return True if this message indicates that there are async events waiting to be sent
        #by the device
        return (self.raw_message[1] & (1 << RR_duino_message.CMD_ASYNC_BIT))!=0
    
    def is_list(self):
        #return True if message is a list (of sensors/turnouts)...
        return (self.raw_message[2] & (1 << RR_duino_message.ADD_LIST_BIT))!=0

    def is_all(self):
        #return True if it is about ALL sensors/turnouts (only for read or write commands/answer)
        return self.is_list() and (self.raw_message[1] & (1 << RR_duino_message.CMD_ALL_BIT))!=0

    def get_error_code(self):
        if (self.raw_message[-1] & 0x80) == 0:
            return None
        return self.raw_message[-1] & 0x0F

    def set_error_code(self,err):
        self.raw_message.append(0x80+err)

    def get_value(self):
        return (self.raw_message[3] & 0x3F, self.raw_message[3] >> RR_duino_message.SUBADD_VALUE_BIT)
    
    def get_list_of_values(self):
        #return a list of pairs (subaddress,value)
        #only valid for a r/w list command
        l = []
        for c in self.raw_message[3:-1]: #forget last byte (the list stop byte)
            l.append((c & 0x3F,c >> RR_duino_message.SUBADD_VALUE_BIT))
        #debug("list of values",l)
        return l

    def get_all_values(self,nb_values):
        #return a list of values (0/1)
        #only valid for a "read all" sensors or turnouts
        l = []
        byte_index = 3
        b = self.raw_message[byte_index]
        bit_pos = 0
        for i in range(nb_values):
            if bit_pos == 8:
                bit_pos = 0
                byte_index+=1
                b = self.raw_message[byte_index] 
                 #sanity check
                if self.raw_message[byte_index] & 0x80 != 0:
                    #stop: this byte is the error code, not all values are present
                    return l
            l.append((b >> bit_pos) & 0x01) #shift left and keep LSB only
            bit_pos+=1
        #debug("list of all values",l)
        return l
        
    def get_sensor_config(self,index):
        #return a tuple (subaddress,pin,type)
        #only valid for a config sensor list command or a show sensor command answer

        if self.raw_message[index] & (1 << RR_duino_message.SUBADD_SENSOR_IO_BIT) != 0:
            sensor_type = RR_duino_message.OUTPUT_SENSOR
        else:
            if self.raw_message[index+1] & (1 << RR_duino_message.PIN_PULLUP_BIT) != 0:
                sensor_type = RR_duino_message.INPUT_SENSOR_PULLUP
            else:
                sensor_type = RR_duino_message.INPUT_SENSOR
        return (self.raw_message[index] & 0x3F, self.raw_message[index+1] & 0x7F, sensor_type)
    
    def get_list_of_sensors_config(self):
        #return a list if tuples (see get_sensor_config)
        l = []
        index = 3 #beginning of list
        while index<len(self.raw_message)-1:  #end of list
            l.append(self.get_sensor_config(index))
            index+=2  #next sensor config
        return l
        
    def get_turnout_config(self,index):
        #return a tuple (subaddress,servo_pin,straight pos,thrown pos [,relay pin 1, relay pin 2,pulse pin 1, pulse pin 2])
        #only valid for a config turnout list command or a show turnout command answer
        subadd = self.raw_message[index] & 0x3F
        if self.raw_message[index] & (1 << RR_duino_message.SUBADD_TURNOUT_RELAY_PINS_BIT) != 0:
            #relay pins are present
            pulse_pin_1 = self.raw_message[index+4] & (1 << RR_duino_message.PIN_PULSE_BIT) != 0
            pulse_pin_2 = self.raw_message[index+5] & (1 << RR_duino_message.PIN_PULSE_BIT) != 0
            return (subadd,self.raw_message[index+1],
                    self.raw_message[index+2],self.raw_message[index+3],
                    self.raw_message[index+4] & 0x7F,self.raw_message[index+5] & 0x7F,
                    pulse_pin_1,pulse_pin_2)
        
        else:
            return (subadd,self.raw_message[index+1],
                    self.raw_message[index+2],self.raw_message[index+3])
        
    def get_list_of_turnouts_config(self):
        #return a list of tuples (subaddress,pin,type)
        #only valid for a config sensor list command or a show sensor command answer
        l = []
        index = 3 #beginning of list
        while index<len(self.raw_message)-1:  #end of list
            turnout_cfg = self.get_turnout_config(index)
            l.append(turnout_cfg)
            if len(turnout_cfg)==4:#next turnout config index depends on relay pins present or not
                index+=4  
            else:
                index+=6
        return l

    def to_wire_message(self):
        if self.raw_message == None:
            return ""
        wire_msg=""
        for b in self.raw_message:
            wire_msg += hex_int(b)+" "

        return wire_msg[:-1]

    @staticmethod
    def wire_to_raw_message(msg):
        """
        decode the message gotten from the wire (same format as cmri raw message except 
        all numbers are hexadecimal strings and space separated)
        transform it as a raw msg
        """
        raw_msg = b""
        byte_list = msg.split(' ')
        for b in byte_list:
            raw_msg += bytes((int(b,16),))
        return raw_msg
    
    @staticmethod
    def from_wire_message(msg):
        return RR_duino_message(RR_duino_message.wire_to_raw_message(msg))

    @staticmethod
    def build_rw_cmd_header(address,read,for_sensor,is_list,is_all=False):
        """
        build the command and address bytes for a rw command
        """
        #set command bit
        command = 1
        #set write bit if needed
        if not read:
            command |= 1 << RR_duino_message.CMD_RW_BIT
        #set turnout bit if needed
        if not for_sensor:
            command |= 1 << RR_duino_message.CMD_SENSOR_TURNOUT_BIT
        #set "all" bit if needed
        if is_all:
            command |= 1 << RR_duino_message.CMD_ALL_BIT
        final_add = address
        if is_list or is_all:
            final_add |= 1 << RR_duino_message.ADD_LIST_BIT
        m = RR_duino_message()
        m.set_header(command,final_add)
        return m

    @staticmethod
    def encode_subadd_value(pair):
        if pair[1] is None or pair[1]==0:
            return bytes((pair[0],))
        else:
            return bytes((pair[0] | (1 << RR_duino_message.SUBADD_VALUE_BIT),))

    @staticmethod
    def encode_sensor_config(config):
        subadd,pin,sensor_type = config
        if sensor_type == RR_duino_message.OUTPUT_SENSOR:
            subadd |= (1 << RR_duino_message.SUBADD_SENSOR_IO_BIT)
        elif sensor_type == RR_duino_message.INPUT_SENSOR_PULLUP:
            pin |= (1 << RR_duino_message.PIN_PULLUP_BIT)
        return bytes((subadd,pin))

    @staticmethod
    def encode_turnout_config(config):
        #config = (subadd,servo_pin,straight_pos,thrown_pos [,relay_pin_1,relay_pin2,pulse pin 1,pulse pin 2])
        subadd = config[0]
        if len(config)>4:
            subadd |= 1 << RR_duino_message.SUBADD_TURNOUT_RELAY_PINS_BIT
        res = bytearray()
        res.extend((subadd,config[1],config[2],config[3]))
        if len(config)>4:
            relay_pins = [config[4]]
            if config[6]:
                relay_pins[0] |= 1 << RR_duino_message.PIN_PULSE_PIN_BIT
            relay_pin.append(config[5])
            if config[7]:
                relay_pins[1] |= 1 << RR_duino_message.PIN_PULSE_PIN_BIT
            res.extend((relay_pins))
        return res

    @staticmethod
    def build_load_from_eeprom(add):
        return RR_duino_message(bytes((0xFF,0b10111001,add)))

    @staticmethod
    def build_save_to_eeprom(add):
        return RR_duino_message(bytes((0xFF,0b10101001,add)))

    @staticmethod
    def build_version_cmd(add):
        return RR_duino_message(bytes((0xFF,0b10001001,add)))

    @staticmethod
    def build_show_cmd(add, on_turnout=False):
        c = 0b11001001
        if on_turnout:
            c |= (1 << RR_duino_message.CMD_SENSOR_TURNOUT_BIT)
        return RR_duino_message(bytes((0xFF,c,add)))

    @staticmethod
    def build_async_cmd(add):
        return RR_duino_message(bytes((0xFF,0b00000101,add)))

    @staticmethod
    def build_simple_rw_cmd(add,subadd,read=True,for_sensor=True,value=None):
        msg = RR_duino_message.build_rw_cmd_header(add,read,for_sensor,False)
        msg.raw_message.extend(RR_duino_message.encode_subadd_value((subadd,value)))
        return msg
    
    @staticmethod
    def is_complete_message(msg):
        #msg is a bytes array
        #returns True if the message is complete, False if msg is incomplete but valid
        #returns None if message is invalid

        def is_cmd_add_message(msg):
            #checks if the message is only 3 bytes: start,command,address
            if msg.is_answer():
                return False
            #it is not an answer
            if msg.is_special_config_cmd():
                #it is a special config, check the codes
                return msg.get_special_config()!=CMD_TURNOUT_FINE_TUNE
            elif msg.is_read_cmd() and msg.is_all(): #read all command
                return True
            return False
                
        
        def sensors_config_list_complete(msg):
            #sensors config are 2 bytes long, so check if we have a full number of config plus one byte
            if (len(msg.raw_message)-1-3) % 2 == 0:
                #yes so check that the last byte, if it is 0x8x it is complete
                return msg.raw_message[-1] & 0x80 != 0

        def next_turnout_config_pos(msg,index):
            #return the position of the next turnout config
            #index is the position of the current turnout config
            if msg.raw_message[index] & (1 << RR_duino_message.SUBADD_TURNOUT_RELAY_PINS_BIT) != 0:
                return index+6
            else:
                return index+4
            
        def turnouts_config_list_complete(msg):
            last = current = 3 #skip start,command and address byte
            while current < len(msg.raw_message):
                last = current
                current = next_turnout_config_pos(msg,last)
            return msg.raw_message[last] & 0x80 != 0

        if len(msg)==0:
            return False
        #length>0
        if msg[0]!=RR_duino_message.START:
            return None
        if len(msg)<3:
            #print("len(msg)<3")
            return False
        message = RR_duino_message(msg)
        if len(msg)==3:
            return is_cmd_add_message(message)

        #length is > 3
        special_config = None
        if message.is_special_config_cmd():
            special_config = message.get_special_config()
        
        #treat the answer cases (must finish by 0x8x)
        if message.is_answer():
            #exceptions: the show and version commands can have bytes with MSB!=0 before the end
            if special_config is None or (special_config!=RR_duino_message.CMD_VERSION
                                          and special_config!=RR_duino_message.CMD_SHOW_SENSORS
                                          and special_config!=RR_duino_message.CMD_SHOW_TURNOUTS):
                return msg[-1] & 0x80 != 0
            #Treat the case of CMD_VERSION
            if special_config == RR_duino_message.CMD_VERSION:
                return len(msg)>=5 and (msg[-1] & 0x80 != 0)
            #Treat the CMD_SHOW_SENSORS:
            if special_config == RR_duino_message.CMD_SHOW_SENSORS:
                return sensors_config_list_complete(message)
            return turnouts_config_list_complete(message)
        #it is a command of length >=4
        #sensors and turnouts config
        if message.is_config_cmd() and special_config is None and message.raw_message[1] & (1 << CMD_CONFIG_DEL_BIT)==0:
            if message.on_turnout():
                if message.is_list():
                    return turnouts_config_list_complete(message)
                else:
                    return next_turnout_config_pos(message,3)==len(message.raw_message)-1
            else:
                if message.is_list():
                    return sensors_config_list_complete(message)
                else:
                    return len(message.raw_message)==5
            
        if message.is_list():  #list commands must finish by 0x8x (all special cases have been dealt with before
            return message.raw_message[-1] & 0x80!=0

        if special_config== CMD_TURNOUT_FINE_TUNE:
            return len(message.raw_message)==5
        #here only simple commands remain: r/w on one device, delete config of one device
        #so it must be complete (they are all 4 bytes commands)
        return True

class RR_duino_node_desc:
    #default dict to add new nodes to the DB when they have no description
    DEFAULT_JSON = { "fullID":None }

    def __init__(self,desc_dict):
        self.desc_dict = dict(desc_dict)  #(shallow) copy the dict containing the node description
        if not "sensors_ev_dict" in self.desc_dict:
            self.desc_dict["sensors_ev_dict"]={}
        if not "turnouts_ev_dict" in self.desc_dict:
            self.desc_dict["turnouts_ev_dict"]={}
        self.ID = self.desc_dict["fullID"]

    def to_json(self):
        return self.desc_dict

class RR_duino_node:
    """
    represents a RR_duino node which means it is an openlcb node (with memory, alias and so on
    and also is linked to the real hardware (via the bus program helper) using the RR_duino protocol
    """
        
    def __init__(self,address,hwversion,desc):

        self.address = address
        self.hwversion = hwversion
        #dict subaddresses <-> config
        self.sensors_cfg={}
        self.turnouts_cfg={}
        #dictionnaries: subaddress <-> corresponding events list
        self.sensors_ev_dict = {}
        self.turnouts_ev_dict= {}
        self.desc = desc
        debug("rr_duino constructor, desc=",desc.desc_dict)
        self.client=client

    def __str__(self):
        res = "RR-duino Node, add="+str(self.address)+",version="+str(self.hwversion)
        return res

    def check_defer(self):
        for defer in self.defer_rw:
            msg = defer.check_time()
            if msg is not None:
                defer.reset()
                self.client.queue(msg.to_wire_message().encode('utf-8'))            

    def generate_events(self,subadd_values,turnouts = False):
        #debug("generate events",subadd_values,turnouts)
        ev_lst=[]
        #first check for the waiting producer identified
        index_to_delete=collections.deque()
        for (subadd,value) in subadd_values:
            for (ev_turnout,ev_subadd,ev_val) in self.waiting_prod_identified:
                if (ev_turnout,subadd)==(turnouts,subadd):
                    if ev_val-2 == value:  #we only use the position reached events part
                        MTI = Frame.MTI_PROD_ID_VAL
                    else:
                        MTI = Frame.MTI_PROD_ID_INVAL
                    debug("producer identified:",subadd,value,turnouts)
                    if ev_turnout:
                        ev_lst.append(Frame.build_from_event(self,self.turnouts_ev_dict[subadd][ev_val],MTI))
                    else:
                        ev_lst.append(Frame.build_from_event(self,self.sensors_ev_dict[subadd][ev_val],MTI))
                    index_to_delete.appendleft(index)
        #delete all read results already used
        for i in index_to_delete:
            subadd_values.pop(i)

        for (subadd,value) in subadd_values:
            if turnouts:
                if subadd in self.turnouts_cfg:
                    if self.turnouts_ev_dict[subadd][value+2]==b"\0"*8:
                        #do not send 0.0.0.0.0.0.0.0 events
                        continue
                    debug("Event for:",subadd,value,turnouts)
                    if self.turnouts_ev_dict[subadd][value+2]==b"\0"*8:
                        #do not send 0.0.0.0.0.0.0.0 events
                        continue
                    #we use value+2 to send the event "turnout has reached position value"
                    ev_lst.append(Frame.build_from_event(self,
                                                         self.turnouts_ev_dict[subadd][value+2],
                                                         0x5B4))
            else:
                if subadd in self.sensors_cfg:
                    if self.sensors_ev_dict[subadd][value]==b"\0"*8:
                        #do not send 0.0.0.0.0.0.0.0 events
                        continue
                    debug("Event for:",subadd,value,turnouts)
                    if self.sensors_ev_dict[subadd][value]==b"\0"*8:
                        #do not send 0.0.0.0.0.0.0.0 events
                        continue
                    ev_lst.append(Frame.build_from_event(self,
                                                         self.sensors_ev_dict[subadd][value],
                                                         0x5B4))
        return ev_lst
    
    def process_receive(self,msg):
        #debug("process receive=",msg.to_wire_message())

        if not msg.is_answer():
            debug("Broken protocol, the bus is receiving a command msg from the slaves!")
            return []
        if msg.get_error_code()!=0:
            debug("Command error!")
            return []
        if msg.is_read_cmd():
            if not msg.is_list():
                return self.generate_events((msg.get_value()),msg.on_turnout())
            elif not msg.is_all():
                return self.generate_events(msg.get_list_of_values(),msg.on_turnout())
            else:
                debug("Read all not implemented yet")
        elif msg.is_write_cmd():
            #just check the error status
            if msg.get_error_code()!=0:
                debug("Last write on node",self.ID,"has failed, error code",msg.get_error_code())
        return []
        
    def consume_event(self,ev,path=None):
        index = 0
        for subadd in self.sensors_ev_dict:
            ev_pair = self.sensors_ev_dict[subadd]
            val = -1
            if ev.id == ev_pair[0]:
                val = 0
            elif ev.id == ev_pair[1]:
                val = 1
            if val>=0:
                if self.sensors_cfg[subadd][1]==RR_duino_message.OUTPUT_SENSOR:
                    debug("RR_duino node",self.desc.desc_dict["fullID"],"sensors consuming event",str(ev))
                    self.client.queue(RR_duino_message.build_simple_rw_cmd(self.address,
                                                                           subadd,
                                                                           False,
                                                                           True,
                                                                           val).to_wire_message().encode('utf-8'))
                else:
                    debug("Error: received an event on an input sensors for RR_duino node",
                          self.desc.desc_dict["fullID"])
        for subadd in self.turnouts_ev_dict:
            ev_quad = self.turnouts_ev_dict[subadd]
            found = False
            for val in range(4):
                print(ev_quad[val],"  ",ev.id)
                if ev.id == ev_quad[val]:
                    found = True
                    break
            debug("turnouts consume event val=",val,found)
            if found:
                if val<2:
                    debug("RR_duino node",self.desc.desc_dict["fullID"],
                          "turnouts consuming event",str(ev))
                    self.client.queue(RR_duino_message.build_simple_rw_cmd(self.address,
                                                                           subadd,
                                                                           False,
                                                                           False,
                                                                           val).to_wire_message().encode('utf-8'))
                else:
                    debug("Error: received an event on an turnouts inputs for RR_duino node",
                          self.desc.desc_dict["fullID"])

    def check_id_producer_event(self,ev):
        """
        check if the event ev is coherent with one input state
        This is used to reply to "identify producer" event
        Return None and will send the answer later
        """
        for subadd in self.sensors_ev_dict:
            ev_pair= sensors_ev_dict[subadd]
            if self.sensors_cfg[subadd][1]!=OUTPUT_SENSOR: #must be an input
                val = -1
                if ev.id == ev_pair[0]:
                    val = 0
                elif ev.id == ev_pair[1]:
                    val = 1
                if val!=-1:
                    #found the input corresponding to the event
                    #place a deferred read and register the event to be answered later
                    self.defer_rw[RR_duino_node.DEFER_READ_SENSORS].add(subadd)
                    self.waiting_prod_identified.append((False,subadd,val))
        for subadd in self.turnouts_ev_dict:
            found = False
            ev_quad = self.turnouts_ev_dict[subadd]
            for val in range(2,4):
                if ev.id == ev_quad[val]:
                    found=True
                    break
            if found:
                #only for the event indicating that the turnout has reached its position
                #place a deferred read and register the event to be answered later
                self.defer_rw[RR_duino_node.DEFER_READ_TURNOUTS].add(subadd)
                self.waiting_prod_identified.append((True,subadd,val))
        return None

    def check_id_consumer_event(self,ev):
        #FIXME
        return openlcb_nodes.Node.ID_PRO_CON_UNKNOWN
def find_node_from_add(add,nodes):
    for n in nodes:
        if n.address == add:
            return n
    return None

def hex_int(i):   #same as hex but withouth the leading "0x"
    return hex(i)[2:]

print("bonjour")
print(RR_duino_message.build_show_cmd(1).to_wire_message())
