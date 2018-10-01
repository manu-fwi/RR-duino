import serial_bus,time,sys
from picotui.context import Context
from picotui.screen import Screen
from picotui.widgets import *
from picotui.defs import *


ANSWER_TIMEOUT = 0.5

def status():
    print("Bus port:",s.ser_port.port," port speed:",s.ser_port.baudrate)
    print("answers:")
    for m in messages:
        print(m)
    for i in range(len(messages)-1,-1,-1):
        print("deleting index",i," len = ",len(messages))
        messages.pop(i)

def set_status(status):
    global device_status
    
    device_status.t=status
    device_status.redraw()

def set_eeprom_status():
    global eeprom_status,eeprom_state
    
    if not eeprom_state[0]:
        result = "Not loaded"
    else:
        result = "Loaded"
    if not eeprom_state[1]:
        result+=" / Not storing"
    else:
        result+=" / Storing"
    eeprom_status.t=result
    eeprom_status.redraw()
    
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

def get_address():
    global address,address_entry,eeprom_state
    try:
        new_address = int(address_entry.get())
    except:
        new_address =address
    if new_address!=address:
        address = new_address
        set_status("Unknown state")
        eeprom_state=[None,None]
        set_eeprom_status()
        
def load_eeprom_clicked(b):
    if check_connection():
        s.send(bytes((0xFF,0b10111001,address)))
        answer,status = wait_for_answer(3)
        set_status(status)
        eeprom_state[0] = answer is not None
        set_eeprom_status()
        
def clear_eeprom_clicked(b):
    if check_connection():
        s.send(bytes((0xFF,0b11111001,address)))
        set_status(wait_for_answer(3)[1])
        
def store_eeprom_clicked(b):
    if check_connection():
        s.send(bytes((0xFF,0b10101001,address)))
        answer,status = wait_for_answer(3)
        eeprom_state[1] = answer is not None
        set_status(status)
        set_eeprom_status()

def set_add():
    #Set address
    s.send(bytes((0xFF,0b10011001,address)))
    return wait_for_answer(3)[1]

def get_version():
    global device_version
    s.send(bytes((0xFF,0b10001001,address)))
    answer,msg = wait_for_answer(3)
    if answer is not None:
        device_version.t=str(answer[0])
        device_version.redraw()
    return msg

def config_cmd(cmd):
    add = check_add(cmd[1])
    if add is not None:
        if cmd[3]=="S": #sensor
            #config sensor subadd=1, input, pin 3, pullup
            subadd = int(cmd[2])
            if cmd[4]=="O":
                subadd |= (1 << 6)
            pin = int(cmd[5])
            if len(cmd) == 7 and cmd[4]=="I" and cmd[6]=="P":
                pin |= (1 << 7)
            s.send(bytes((0xFF,0b10000001,add,subadd,pin)))
        elif cmd[3]=="T": #turnout
            subadd = int(cmd[2])
            if len(cmd)>7:
                subadd |= 1 << 6
                #config turnouts with relays
                relay_pin_1 =int(cmd[7])
                if len(cmd)==9:
                    relay_pin_2 = int (cmd[8])
                else:
                    relay_pin_2 = 254
                s.send(bytes((0xFF,0b10010001,add,subadd,int(cmd[4]),int(cmd[5]),int(cmd[6]),relay_pin_1,relay_pin_2)))
            else:
                s.send(bytes((0xFF,0b10010001,add,subadd,int(cmd[4]),int(cmd[5]),int(cmd[6]))))
                

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


#class MainForm(npyscreen.Form):
#    SERIAL_VALID = 1
#    LOAD_EEPROM = 2
#    SAVE_EEPROM = 3
#    CLEAR_EEPROM = 4
#    SET_ADDRESS = 5
#    def create(self):
#        self.box_serial = self.add(SerialPortBox,name="Serial Port",max_height=6,scroll_exit=True)
#        self.nextrely+=1
#        self.wg_address = self.add(npyscreen.TitleText, name = "Device address",value = str(address))
#        self.wg_save_state=self.add(npyscreen.TitleFixedText, name = "Saving to eeprom", value ="NO")
#        self.add(ButtonPressEvent,
#                 name= "Set device address (pin 2 must be held to ground)",
#                 event="set_address")        
#        self.add(ButtonPressEvent,name= "Load From EEPROM",event="load_eeprom")
#        self.add(ButtonPressEvent,name= "Store to EEPROM",event="store_to_eeprom")
#        self.add(ButtonPressEvent,name= "CLEAR EEPROM",event="clear_eeprom")
#        self.wg_answer_status = self.add(npyscreen.TitleFixedText,name = "Answer status",value="OK")
#        
#        self.nextrely+=1
#        self.wg_sensor_turnout =  self.add(npyscreen.TitleSelectOne, max_height=3, value = [0,], name="Subdevice type",
#                                           values = ["Turnout","Sensor"], scroll_exit=True)
#
#    def serial_port_connect(self,event):
#        print("EVENT")
#        if self.box_serial.button_is_connect():
#            try:
#                s.start()
#                self.box_serial.connect_button(False)
#            except:
#                s.stop()
#        else:
#            s.stop()
#            self.box_serial.connect_button(False)
#
#    def afterEditing(self):
#        self.parentApp.setNextForm(None)
#
#    def valid(self,code):
#        global serial_port,serial_speed,s
#        print(code)
#        if code==MainForm.SERIAL_VALID:
#            new_serial_port = self.wg_serial_port.value
#            try:
#                new_serial_baud = int(self.wg_serial_baud.value)
#                print(new_serial_baud)
#            except:
#                new_serial_baud = serial_speed
#            if new_serial_port != serial_port or new_serial_baud!=serial_speed:
#                s.stop()
#                serial_port=new_serial_port
#                serial_speed=new_serial_baud
#                s = serial_bus.serial_bus(serial_port,serial_speed)
#                s.start()
#        elif code == MainForm.LOAD_EEPROM:
#            load_from_eeprom()
#        elif code == MainForm.SAVE_EEPROM:
#            store_to_eeprom()
#            self.wg_save_state.value="YES"
#            self.wg_save_state.update()
#        elif code == MainForm.SET_ADDRESS:
#            set_add()
#    def while_waiting(self):
#        pass

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
    global connect_state
    if connect_state:  #was connected
        connect_button.t="Connect"
        s.stop()
        connect_state = False
    else:
        #try:
            s.start()
            connect_button.t="Disconnect"
            connect_state = True
        #except:
            pass
    connect_button.redraw()

def check_connection():
    if not connect_state:
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
    global next_dialog
    next_dialog = turnout_dialog
    main_dialog_w.exit = True

def sensor_clicked(b):
    global next_dialog,main_dialog_w
    if not check_connection():
        return
    next_dialog = sensor_dialog
    main_dialog_w.exit = True

    
def main_dialog():
    global next_dialog,device_status,device_version,eeprom_status,connect_state,connect_button,address_entry
    global main_dialog_w
    
    Screen.attr_color(C_WHITE, C_BLUE)
    Screen.cls()
    Screen.attr_reset()
    main_dialog_w = DialogNew(5, 5, 70, 15,title="RR-DUINO SETUP")
    next_dialog = None

    main_dialog_w.add(1, 1, WFrame(35, 5, "Serial Port"))
    main_dialog_w.add(2, 2, "Port:")
    port_entry = WTextEntry(20, serial_port)
    main_dialog_w.add(11, 2, port_entry)
    main_dialog_w.add(2, 3, "Speed:")
    baudrate_entry = WTextEntry(20, str(serial_speed))
    main_dialog_w.add(11, 3,baudrate_entry)
    connect_button = WButton(12,"Connect")
    connect_state = False #begin disconnected
    connect_button.on("click",connect_clicked)
    main_dialog_w.add(11,4,connect_button)

    main_dialog_w.add(36,1,WFrame(35,5,"Device"))
    main_dialog_w.add(37,2,"Address:")
    address_entry = WTextEntry(20, str(address))
    main_dialog_w.add(46, 2, address_entry)
    set_address_button = WButton(13,"Set address")
    main_dialog_w.add(37,3,set_address_button)
    set_address_button.on("click",set_address_clicked)
    check_device_button = WButton(15,"Check device")
    check_device_button.on("click",check_device_clicked)
    main_dialog_w.add(51,3,check_device_button)
    main_dialog_w.add(37,4,"Version:")
    device_version = WLabel("Unknown",23)
    main_dialog_w.add(45,4,device_version)
    
    main_dialog_w.add(1,6,WFrame(63,4,"Device setup"))

    main_dialog_w.add(2,7,"Status:")
    device_status = WLabel("Unknown state",26)
    main_dialog_w.add(10,7,device_status)
    eeprom_status = WLabel("Not loaded / Not storing",25)
    main_dialog_w.add(37,7,eeprom_status)
    load_eeprom_button = WButton(15,"Load EEPROM")
    load_eeprom_button.on("click",load_eeprom_clicked)
    main_dialog_w.add(2,8,load_eeprom_button)
    store_eeprom_button = WButton(15,"Store to EEPROM")
    store_eeprom_button.on("click",store_eeprom_clicked)
    main_dialog_w.add(18,8,store_eeprom_button)
    clear_eeprom_button = WButton(15,"Clear EEPROM")
    clear_eeprom_button.on("click",clear_eeprom_clicked)
    main_dialog_w.add(34,8,clear_eeprom_button)

    main_dialog_w.add(1,10,WFrame(63,3,"Setup"))
    turnout_button = WButton(15,"Turnouts setup")
    turnout_button.on("click",turnout_clicked)
    main_dialog_w.add(2,11,turnout_button)
    sensor_button = WButton(15,"Sensors setup")
    sensor_button.on("click",sensor_clicked)
    main_dialog_w.add(18,11,sensor_button)
    main_dialog_w.loop()

def sensor_commit_clicked(b):
    pass

def load_sensors():
    pass
    
def sensor_dialog():
    global sensor_subadd_entry,sensor_pin_entry,sensor_io
    Screen.attr_color(C_WHITE, C_BLUE)
    Screen.cls()
    Screen.attr_reset()
    sensor_dialog_w = DialogNew(1, 1, 70, 20,title="RR-DUINO SENSOR SETUP")

    sensor_dialog_w.add(1,1,WFrame(58,3,"Device setup address "+str(address)))

    sensor_dialog_w.add(2,2,"Status:")
    device_status = WLabel("Unknown state",22)
    sensor_dialog_w.add(10,2,device_status)
    eeprom_status = WLabel("Not loaded / Not storing",25)
    sensor_dialog_w.add(32,2,eeprom_status)

    sensor_dialog_w.add(1,4,WFrame(25,8,"New sensor"))
    sensor_dialog_w.add(2,5,"Subaddress:")
    sensor_subadd_entry=WTextEntry(5,"0")
    sensor_dialog_w.add(14,5,sensor_subadd_entry)
    sensor_dialog_w.add(2,6,"Pin:")
    sensor_pin_entry=WTextEntry(5,"0")
    sensor_dialog_w.add(14,6,sensor_pin_entry)
    sensor_io = WRadioButton(["input","input with pull-up","output"])
    sensor_dialog_w.add(2,7,sensor_io)
    sensor_commit_button = WButton(15,"Commit sensor")
    sensor_commit_button.on("click",sensor_commit_clicked)
    sensor_dialog_w.add(5,10,sensor_commit_button)

    sensor_dialog_w.loop()
    next_dialog = None

def turnout_dialog():
    pass
    
messages=[]
serial_port = "/dev/ttyUSB1"
serial_speed = 19200
s = serial_bus.serial_bus(serial_port,serial_speed)
address=0
subaddress = 0
turnout = False
next_dialog = None
device_status = device_version = connect_state = connect_button = eeprom_status = main_dialog_w = None
sensor_pin_entry = sensor_io = sensor_subadd_entry = None
eeprom_state = [False,False]
while True:
    with Context():
        main_dialog()
        if next_dialog is None:
            break
        next_dialog()
        
s.stop()

#while not done:
#    read_sockets=wait_for_clients()
#    for sock in read_sockets:
#        m=""
#        try:
#            m = sock.recv(200).decode('utf-8')
#        except socket.error:
#            print("recv error")
#        if m!="":
#            print("received:",m)
#            cmd = m.split()
#            if cmd[0]=="L":
#                load_from_eeprom(cmd)
#            elif cmd[0]=="E":
#                store_to_eeprom(cmd)
#            elif cmd[0]=="A":
#                async_events(cmd)
#            elif cmd[0]=="S":
#                show(cmd)
#            elif cmd[0]=="R":
#                read_cmd(cmd)
#            elif cmd[0]=="W":
#                write_cmd(cmd)
#            elif cmd[0]=="C":
#                config_cmd(cmd)
#            elif cmd[0]=="Y":
#                set_add(cmd)
#            elif cmd[0]=="V":
#                get_version(cmd)
#            elif cmd[0]=="Q":
#                clear_eeprom(cmd)
#            else:
#                print("Unknown command:",m)
#            status()
#    s.process_IO()
#    m= s.read()
#    if m is not None:
#        if m[0]==0xFF or len(messages)==0:
#            messages.append(m)
#        else:
#            messages[len(messages)-1]+=m
#
#s.ser_port.close()
#
