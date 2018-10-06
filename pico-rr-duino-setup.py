import serial_bus,time,sys
from picotui.context import Context
from picotui.screen import Screen
from picotui.widgets import *
from picotui.defs import *


ANSWER_TIMEOUT = 0.5
TURNOUT_TUNE_TIMEOUT = 5

def set_status(status):    
    main_data.device_status.t=status
    main_data.device_status.redraw()

def set_eeprom_status(eeprom_status):
    
    if not main_data.eeprom_state[0]:
        result = "Not loaded"
    else:
        result = "Loaded"
    if not main_data.eeprom_state[1]:
        result+=" / Not storing"
    else:
        result+=" / Storing"
    eeprom_status.t=result
    
def wait_for_answer(min_length):
    m=b""
    beg = time.time()
    while (len(m)<min_length or m[-1]&0x80==0) and time.time()<beg+ANSWER_TIMEOUT:
        s.process_IO()
        if s.available():
            m+=s.read()
            beg = time.time()
    if len(m)==0:
        return (None,"Device not responding")
  
    elif m[-1]&0x7F!=0:
        return (None,"Error "+str(m[-1]&0x7F))
 
    else:
        return (m[3:],"OK")
    
def decode(m):
    res=""
    for b in m:
        res += " "+str(b)
    return res

def wait_for_show_answer(turnouts=False):
    m=b""
    beg = time.time()
    next_subadd_pos = 3
    while time.time()<beg+ANSWER_TIMEOUT:
        s.process_IO()
        if s.available():
            m+=s.read()
            beg = time.time()
            if len(m)==next_subadd_pos+1 and (m[next_subadd_pos] & 0x80!=0):
                #end of the message
                break
            elif len(m)>next_subadd_pos:
                #compute next subbaddress pos
                if not turnouts:
                    next_subadd_pos+=2
                else:
                    #check if relay pins are needed for this turnout
                    if (m[next_subadd_pos] & (1<<6))!=0:
                        next_subadd_pos+=6
                    else:
                        next_subadd_pos+=4

    if len(m)==0:
        return (None,"Device not responding")
  
    elif m[-1]&0x7F!=0:
        return (None,"Error "+str(m[-1]&0x7F))
 
    else:
        return (m,"OK")

def get_address():
    global address
    try:
        new_address = int(main_data.address_entry.get())
    except:
        new_address =address
    if new_address!=address:
        address = new_address
        set_status("Unknown state")
        main_data.eeprom_state=[None,None]
        set_eeprom_status(main_data.eeprom_status)
        main_data.eeprom_status.redraw()
        
def load_eeprom_clicked(b):
    if check_connection():
        s.send(bytes((0xFF,0b10111001,address)))
        answer,status = wait_for_answer(3)
        set_status(status)
        main_data.eeprom_state[0] = answer is not None
        set_eeprom_status(main_data.eeprom_status)
        main_data.eeprom_status.redraw()
        
        
def clear_eeprom_clicked(b):
    if check_connection():
        s.send(bytes((0xFF,0b11111001,address)))
        set_status(wait_for_answer(3)[1])
        
def store_eeprom_clicked(b):
    if check_connection():
        s.send(bytes((0xFF,0b10101001,address)))
        answer,status = wait_for_answer(3)
        main_data.eeprom_state[1] = answer is not None
        set_status(status)
        set_eeprom_status(main_data.eeprom_status)
        main_data.eeprom_status.redraw()

def set_add():
    #Set address
    s.send(bytes((0xFF,0b10011001,address)))
    return wait_for_answer(3)[1]

def get_version():
    s.send(bytes((0xFF,0b10001001,address)))
    answer,msg = wait_for_answer(3)
    if answer is not None:
        main_data.device_version.t=str(answer[0])
        main_data.device_version.redraw()
    return msg

def async_events(cmd):
#send async event command:
    add = check_add(cmd[1])
    if add is not None:
        c = 0b00000101
        s.send(bytes((0xFF,c,add)))

def show(cmd):
    add = check_add(cmd[1])
    if add is not None:
        c= 0b11001001
        if cmd[2]=="T":
            c|= (1 << 4)
        s.send(bytes((0xFF,c,add)))

def read_cmd(cmd):
    add = check_add(cmd[1])
    if add is not None:
        print("cmd[2]=",cmd[2])
        c = 0b00000001
        if cmd[2]=='A':
            subadd = None
            c |= (1<<6)
            add |= (1<<6)
        else:
            subadd = int(cmd[2])
        if cmd[3]=="T":
            c |= (1<<4)
        if subadd is not None:
            s.send(bytes((0xFF,c,add,subadd)))
        else:
            s.send(bytes((0xFF,c,add)))

def write_cmd(cmd):
    add = check_add(cmd[1])
    if add is not None:
        subadd = int(cmd[2])
        if cmd[4]=="1":
            subadd |= (1<<6)
        c = 0b00100001
        if cmd[3]=="T":
            c |= (1<<4)
        s.send(bytes((0xFF,c,add,subadd)))


class DialogNew(Dialog):
    def __init__(self, x, y, w=0, h=0, title=""):
        super().__init__(x,y,w,h,title)
        self.exit = False
        
    def loop(self):
        self.redraw()
        while True:
            key = self.get_input()
            res = self.handle_input(key)

            if res is not None and res is not True:
                return res
            if self.exit:
                return ACTION_CANCEL
            self.idle()
    def idle(self):
        pass

def connect_clicked(b):
    if main_data.connect_state:  #was connected
        main_data.connect_button.t="Connect"
        s.stop()
        connect_state = False
    else:
        #try:
            s.start()
            main_data.connect_button.t="Disconnect"
            main_data.connect_state = True
        #except:
            pass
    main_data.connect_button.redraw()

def check_connection():
    if not main_data.connect_state:
        set_status("Serial port not connected!")
        return False
    get_address()
    if address==0:
        set_status("Address 0 is invalid!")
        return False
    return True
    
def check_device_clicked(b):
    if check_connection():
        set_status("Checking device...")
        set_status(get_version())

def set_address_clicked(b):
    if check_connection():
        set_status(set_add())

def debug(*args,**keywords):
    pass

def turnout_clicked(b):
    global next_dialog,main_data
    if not check_connection():
        return
    next_dialog = turnout_dialog
    main_data.dialog_w.exit = True

def sensor_clicked(b):
    global next_dialog,main_data
    if not check_connection():
        return
    next_dialog = sensor_dialog
    main_data.dialog_w.exit = True


class MainDialogData:
    def __init__(self):
        self.device_status = self.eeprom_status = None
        self.device_version = self.address_entry = self.port_entry = self.baudrate_entry = None
        self.connect_state = False
        self.connect_button = None
        self.dialog_w = None
        self.eeprom_state = [False,False]

        
def main_dialog():
    global main_data
    global next_dialog
    
    Screen.attr_color(C_WHITE, C_BLUE)
    Screen.cls()
    Screen.attr_reset()
    main_data.dialog_w = DialogNew(5, 5, 70, 15,title="RR-DUINO SETUP")
    next_dialog = None

    main_data.dialog_w.add(1, 1, WFrame(35, 5, "Serial Port"))
    main_data.dialog_w.add(2, 2, "Port:")
    main_data.port_entry = WTextEntry(20, serial_port)
    main_data.dialog_w.add(11, 2, main_data.port_entry)
    main_data.dialog_w.add(2, 3, "Speed:")
    main_data.baudrate_entry = WTextEntry(20, str(serial_speed))
    main_data.dialog_w.add(11, 3,main_data.baudrate_entry)
    main_data.connect_button = WButton(12,"Connect")
    main_data.connect_state = False #begin disconnected
    main_data.connect_button.on("click",connect_clicked)
    main_data.dialog_w.add(11,4,main_data.connect_button)

    main_data.dialog_w.add(36,1,WFrame(35,5,"Device"))
    main_data.dialog_w.add(37,2,"Address:")
    main_data.address_entry = WTextEntry(20, str(address))
    main_data.dialog_w.add(46, 2, main_data.address_entry)
    set_address_button = WButton(13,"Set address")
    main_data.dialog_w.add(37,3,set_address_button)
    set_address_button.on("click",set_address_clicked)
    check_device_button = WButton(15,"Check device")
    check_device_button.on("click",check_device_clicked)
    main_data.dialog_w.add(51,3,check_device_button)
    main_data.dialog_w.add(37,4,"Version:")
    main_data.device_version = WLabel("Unknown",23)
    main_data.dialog_w.add(45,4,main_data.device_version)
    
    main_data.dialog_w.add(1,6,WFrame(63,4,"Device setup"))

    main_data.dialog_w.add(2,7,"Status:")
    main_data.device_status = WLabel("Unknown state",26)
    main_data.dialog_w.add(10,7,main_data.device_status)
    main_data.eeprom_status = WLabel("Not loaded / Not storing",25)
    main_data.dialog_w.add(37,7,main_data.eeprom_status)
    load_eeprom_button = WButton(15,"Load EEPROM")
    load_eeprom_button.on("click",load_eeprom_clicked)
    main_data.dialog_w.add(2,8,load_eeprom_button)
    store_eeprom_button = WButton(15,"Store to EEPROM")
    store_eeprom_button.on("click",store_eeprom_clicked)
    main_data.dialog_w.add(18,8,store_eeprom_button)
    clear_eeprom_button = WButton(15,"Clear EEPROM")
    clear_eeprom_button.on("click",clear_eeprom_clicked)
    main_data.dialog_w.add(34,8,clear_eeprom_button)

    main_data.dialog_w.add(1,10,WFrame(63,3,"Setup"))
    turnout_button = WButton(15,"Turnouts setup")
    turnout_button.on("click",turnout_clicked)
    main_data.dialog_w.add(2,11,turnout_button)
    sensor_button = WButton(15,"Sensors setup")
    sensor_button.on("click",sensor_clicked)
    main_data.dialog_w.add(18,11,sensor_button)
    main_data.dialog_w.loop()

def pad_int(int,nb):
    s = str(int)
    if len(s)<nb:
        s=" "*(nb-len(s))+s
    return s

def get_sensors_cfg(msg):
    pos = 0
    while pos+1<len(msg):
        output = (msg[pos] & (1<<6))!=0
        subadd = msg[pos] & 0x3F
        pos+=1
        if not output:
            pullup = (msg[pos] & (1<<7))!=0
        pin = msg[pos]&0x7F
        pos+=1
        s = "subadd:"+pad_int(subadd,2)+"|pin:"+pad_int(pin,3)+"|"
        if output:
            s+="O"
        else:
            s+="I"
            if pullup:
                s+=" P"
        sensor_data.sensors_list.append(s)
    
def load_sensors():
    sensor_data.sensors_list = []
    done = False
    while not done:
        #send command
        c= 0b11001001
        s.send(bytes((0xFF,c,address)))
        msg,code = wait_for_show_answer()
        sensor_data.device_status.t=code
        if msg is None:
            done=True
        else:
            #check if there is another answer pending
            done = (msg[1]& (1<<1))==0
            get_sensors_cfg(msg[3:])

class SensorDialogData:
    def __init__(self):
        self.subadd_entry = self.pin_entry=None
        self.io=self.list_wg = None
        self.sensors_list = []
        self.device_status = None
        self.dialog_w = None

def config_sensor(subadd,pin,io_type):
    if io_type==2:
        subadd |= (1<<6)
    elif io_type==1:
        pin |= (1<<7)
    s.send(bytes((0xFF,0b10000001,address,subadd,pin)))
    
def sensor_commit_clicked(b):
    global sensor_data
    error = False
    try:
        subadd = int(sensor_data.subadd_entry.get())
    except:
        error = True

    if subadd<=0 or subadd>=64 or error:
        return
    try:
        pin = int(sensor_data.pin_entry.get())
    except:
        error = True

    if pin<=0 or pin>=128 or error:
        return
    io_type= sensor_data.io.get() #0=input, 1=input w/ pullup, 2=output
    config_sensor(subadd,pin,io_type)
    m,code = wait_for_answer(3)
    if m is None:
        sensor_data.device_status.t = code
        #error
    else:
        #no error
        sensor_data.device_status.t = m
        #reload the list
        load_sensors()
        sensor_data.list_wg.items = sensor_data.sensors_list
        sensor_data.list_wg.set_lines(sensor_data.list_wg.items)
        sensor_data.list_wg.redraw()
    sensor_data.device_status.redraw()
    
def sensor_dialog():
    global sensor_data

    Screen.attr_color(C_WHITE, C_BLUE)
    Screen.cls()
    Screen.attr_reset()
    sensor_data.dialog_w = DialogNew(1, 1, 70, 20,title="RR-DUINO SENSOR SETUP")

    sensor_data.dialog_w.add(1,1,WFrame(58,3,"Device setup address "+str(address)))

    sensor_data.dialog_w.add(2,2,"Status:")
    sensor_data.device_status = WLabel("Unknown state",22)
    sensor_data.dialog_w.add(10,2,sensor_data.device_status)
    sensor_data.eeprom_status = WLabel("Not loaded / Not storing",25)
    set_eeprom_status(sensor_data.eeprom_status)
    sensor_data.dialog_w.add(32,2,sensor_data.eeprom_status)

    sensor_data.dialog_w.add(1,4,WFrame(25,8,"New sensor"))
    sensor_data.dialog_w.add(2,5,"Subaddress:")
    sensor_data.subadd_entry=WTextEntry(5,"0")
    sensor_data.dialog_w.add(14,5,sensor_data.subadd_entry)
    sensor_data.dialog_w.add(2,6,"Pin:")
    sensor_data.pin_entry=WTextEntry(5,"0")
    sensor_data.dialog_w.add(14,6,sensor_data.pin_entry)
    sensor_data.io = WRadioButton(["input","input with pull-up","output"])
    sensor_data.dialog_w.add(2,7,sensor_data.io)
    sensor_commit_button = WButton(15,"Commit sensor")
    sensor_commit_button.on("click",sensor_commit_clicked)
    sensor_data.dialog_w.add(5,10,sensor_commit_button)

    load_sensors()

    sensor_data.list_wg=WListBox(28,6,sensor_data.sensors_list)
    sensor_data.dialog_w.add(26,4,WFrame(30,8,"Sensors"))
    sensor_data.dialog_w.add(27,5,sensor_data.list_wg)

    sensor_data.dialog_w.loop()
    next_dialog = None

class TurnoutDialogData:
    def __init__(self):
        self.subadd_entry = self.servo_pin_entry=self.relay_pins_entry=None
        self.straight_pos_entry = self.thrown_pos_entry = None
        self.pulse_relay_pins = None
        self.list_wg = None
        self.turnouts_list = []
        self.device_status = None
        self.fine_tune_label = None
        self.fine_tune_pos = 90
        self.fine_tune_time = None
        self.dialog_w = None

    def idle(self):
        if self.fine_tune_time is not None:
            if self.fine_tune_time+TURNOUT_TUNE_TIMEOUT<time.time():
                self.fine_tune_pos = 0
                fine_tune_turnout(None)
                self.fine_tune_time = None


def config_turnout(subadd,servo_pin,straight_pos,thrown_pos,pulse_pins,relay_pins):
    if len(relay_pins)>0 and (relay_pins[0]!=254 or relay_pins[1]!=254):
        subadd |= 1 << 6
        for i in range(2):
            if pulse_pins:
                relay_pins[i]|=0x80
        s.send(bytes((0xFF,0b10010001,address,subadd,servo_pin,straight_pos,thrown_pos,relay_pins[0],relay_pins[1])))
    else:
        s.send(bytes((0xFF,0b10010001,address,subadd,servo_pin,straight_pos,thrown_pos)))

def turnout_commit_clicked(b):
    global turnout_data
    error = False
    try:
        subadd = int(turnout_data.subadd_entry.get())
    except:
        error = True

    if subadd<=0 or subadd>=64 or error:
        return
    try:
        servo_pin = int(turnout_data.servo_pin_entry.get())
    except:
        error = True

    if servo_pin<=0 or servo_pin>=126 or error:
        return

    error = False
    try:
        straight_pos = int(turnout_data.straight_pos_entry.get())
    except:
        error = True

    if straight_pos<0 or straight_pos>255 or error:
        return
    try:
        thrown_pos = int(turnout_data.thrown_pos_entry.get())
    except:
        error = True

    if thrown_pos<0 or thrown_pos>255 or error:
        return

    pulse_pins= turnout_data.pulse_relay_pins.choice
    relay_pins = []
    for i in range(2):
        s = turnout_data.relay_pins_entry[i].get()
        if s=="":
            relay_pins.append(254)
        else:
            error=False
            try:
                relay_pins.append(int(s))
            except:
                error=True
            if error:
                return
            if relay_pins[i]<=0 or relay_pins[i]>126:
                return
            if pulse_pins:
                relay_pins[i]|=0x80
                
    config_turnout(subadd,servo_pin,straight_pos,thrown_pos,pulse_pins,relay_pins)
    m,code = wait_for_answer(3)
    if m is None:
        turnout_data.device_status.t = code
        #error
    else:
        #no error
        turnout_data.device_status.t = m
        #reload the list
        load_turnouts()
        turnout_data.list_wg.items = turnout_data.turnouts_list
        turnout_data.list_wg.set_lines(turnout_data.list_wg.items)
        turnout_data.list_wg.redraw()
    turnout_data.device_status.redraw()    

def relay_pin_str(pin):
    if pin == 0 or pin==254:
        return "   "
    else:
        return pad_int(pin,3)
    
def get_turnouts_cfg(msg):
    pos = 0
    while pos+1<len(msg):
        relay_pins = (msg[pos] & (1<<6))!=0
        subadd = msg[pos] & 0x3F
        pos+=1
        pin = msg[pos]
        pos+=1
        straight = msg[pos]
        pos+=1
        thrown = msg[pos]
        pos+=1
        s = "sub:"+pad_int(subadd,2)+"|P:"+pad_int(pin,3)+"|S:"
        s+= pad_int(straight,3)+"|T:"+pad_int(thrown,3)
        if relay_pins:
            pin1=msg[pos] & 0x7F
            pulse = (msg[pos]&0x80)!=0
            pos+=1
            pin2=msg[pos] & 0x7F
            s+="|r1:"+relay_pin_str(pin1)+"|r2:"+relay_pin_str(pin2)+"|"
            if pulse:
                s+="P"
            pos+=1
        else:
            s+="|"+" "*6+"|"+" "*6+"|"
        turnout_data.turnouts_list.append(s)
    

def load_turnouts():
    turnout_data.turnouts_list = []
    done = False
    while not done:
        #send command
        c= 0b11011001
        s.send(bytes((0xFF,c,address)))
        msg,code = wait_for_show_answer(True)
        turnout_data.device_status.t=code
        if msg is None:
            done=True
        else:
            #check if there is another answer pending
            done = (msg[1]& (1<<1))==0
            get_turnouts_cfg(msg[3:])

def sanitize_pos():
    if turnout_data.fine_tune_pos < 0:
        turnout_data.fine_tune_pos=0
    elif turnout_data.fine_tune_pos>180:
        turnout_data.fine_tune_pos=180

def fine_tune_turnout_diff(val):
    turnout_data.fine_tune_pos += val
    sanitize_pos()
    fine_tune_turnout(turnout_data.fine_tune_pos)
    
def fine_tune_turnout_plus(b):
    fine_tune_turnout_diff(1)

def fine_tune_turnout_minus(b):
    fine_tune_turnout_diff(-1)

def fine_tune_turnout_PLUS(b):
    fine_tune_turnout_diff(5)

def fine_tune_turnout_MINUS(b):
    fine_tune_turnout_diff(-5)
    
def fine_tune_turnout(pos):
    turnout_data.fine_tune_time = time.time()
    turnout_str = turnout_data.list_wg.items[turnout_data.list_wg.choice]
    subadd=int(turnout_str[4:6])
    s.send(bytes((0xFF,0b11101001,address,subadd,pos)))
    m,code=wait_for_answer(3)
    turnout_data.device_status.t=code
    if m is not None:
        turnout_data.fine_tune_label.t = pad_int(pos,3)
        turnout_data.fine_tune_label.redraw()
        return True
    return False

def turnout_dialog():
    global turnout_data

    Screen.attr_color(C_WHITE, C_BLUE)
    Screen.cls()
    Screen.attr_reset()
    turnout_data.dialog_w = DialogNew(1, 1, 70, 20,title="RR-DUINO SENSOR SETUP")

    turnout_data.dialog_w.add(1,1,WFrame(58,3,"Device setup address "+str(address)))

    turnout_data.dialog_w.add(2,2,"Status:")
    turnout_data.device_status = WLabel("Unknown state",22)
    turnout_data.dialog_w.add(10,2,turnout_data.device_status)
    turnout_data.eeprom_status = WLabel("Not loaded / Not storing",25)
    set_eeprom_status(turnout_data.eeprom_status)
    turnout_data.dialog_w.add(32,2,turnout_data.eeprom_status)

    turnout_data.dialog_w.add(1,4,WFrame(25,12,"New turnout"))
    turnout_data.dialog_w.add(2,5,"Subaddress:")
    turnout_data.subadd_entry=WTextEntry(5,"0")
    turnout_data.dialog_w.add(16,5,turnout_data.subadd_entry)
    turnout_data.dialog_w.add(2,6,"Servo pin:")
    turnout_data.servo_pin_entry=WTextEntry(5,"0")
    turnout_data.dialog_w.add(16,6,turnout_data.servo_pin_entry)
    turnout_data.dialog_w.add(2,7,"Straight pos:")
    turnout_data.straight_pos_entry=WTextEntry(5,"0")
    turnout_data.dialog_w.add(16,7,turnout_data.straight_pos_entry)
    turnout_data.dialog_w.add(2,8,"Thrown pos:")
    turnout_data.thrown_pos_entry=WTextEntry(5,"0")
    turnout_data.dialog_w.add(16,8,turnout_data.thrown_pos_entry)

    turnout_data.pulse_relay_pins=WCheckbox("Pulse relay pins")
    turnout_data.dialog_w.add(2,10,turnout_data.pulse_relay_pins)
    turnout_data.relay_pins_entry =[]
    turnout_data.dialog_w.add(2,11,"Relay pin 1:")
    turnout_data.relay_pins_entry.append(WTextEntry(5,""))
    turnout_data.dialog_w.add(16,11,turnout_data.relay_pins_entry[0])
    turnout_data.dialog_w.add(2,12,"Relay pin 2:")
    turnout_data.relay_pins_entry.append(WTextEntry(5,""))
    turnout_data.dialog_w.add(16,12,turnout_data.relay_pins_entry[1])

    turnout_commit_button = WButton(15,"Commit turnout")
    turnout_commit_button.on("click",turnout_commit_clicked)
    turnout_data.dialog_w.add(5,14,turnout_commit_button)

    load_turnouts()

    turnout_data.list_wg=WListBox(40,10,turnout_data.turnouts_list)
    turnout_data.dialog_w.add(26,4,WFrame(42,12,"Turnouts"))
    turnout_data.dialog_w.add(27,5,turnout_data.list_wg)

    fine_tune = WButton(5,"<<")
    fine_tune.on("click",fine_tune_turnout_MINUS)
    turnout_data.dialog_w.add(27,16,fine_tune)
    fine_tune = WButton(5,"<")
    fine_tune.on("click",fine_tune_turnout_minus)
    turnout_data.dialog_w.add(33,16,fine_tune)
    turnout_data.fine_tune_label = WLabel(pad_int(90,3),5)
    turnout_data.dialog_w.add(39,16,turnout_data.fine_tune_label)
    fine_tune = WButton(5,">")
    fine_tune.on("click",fine_tune_turnout_plus)
    turnout_data.dialog_w.add(45,16,fine_tune)
    fine_tune = WButton(5,">>")
    fine_tune.on("click",fine_tune_turnout_PLUS)
    turnout_data.dialog_w.add(51,16,fine_tune)
    next_dialog = None
    turnout_data.dialog_w.loop()
    
messages=[]
serial_port = "/dev/ttyUSB1"
serial_speed = 19200
s = serial_bus.serial_bus(serial_port,serial_speed)
address=0
subaddress = 0
turnout = False
next_dialog = None
main_data = MainDialogData()
sensor_data=SensorDialogData()
turnout_data=TurnoutDialogData()
while True:
    with Context():
        main_dialog()
        if next_dialog is None:
            break
        next_dialog()
        
s.stop()
