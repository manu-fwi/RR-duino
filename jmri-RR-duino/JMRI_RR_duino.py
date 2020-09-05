# Based on a script by Geoff Bunza 2018, itself based in part on a script by
# Bob Jacobsen as part of the JMRI distribution
# Version 0.1
# The Automat object creates a separate thread
# that can sit there, waiting for messages from a TCP connection (sensors)
# and sending messages to the TCP connection (turnouts)
# Messages:
#   - turnouts close/throw sends -> RTbb:nn:xxx,0/1
#   - sensors change when receiving RSbb:nn:xxx,0/1
# bb:bus number (you can have several rrduino node controllers
# nn: -units represent the node address on a RR_duino bus minus 1 (RR_duino addresses are in range 1..62) so in range 0..61
# xxx: turnout/sensor sub address on the node minus 1
#       !! turnouts sub addresses are 0-99
#       !! input sensors sub addresses are 0-99
#       !! turnout feedback sensors are 200-299 (must be turnout subaddress + 200)
#       !! outputs as turnouts are 100-199
#       !! output feedbacks are 200-299 (outputs can be read in RRduino useful
#       !! for startup to have a coherent state between jmri and hardware


import jarray
import jmri
#import purejavacomm
import java
import socket
import json

# define a turnout listener that will 
class TurnoutTransfer(java.beans.PropertyChangeListener):
    # initialization 
    # registers to receive events
    def __init__(self, id,jmri_rr_duino) :
        self.name = "ITRT"+str(id)
        turnout = turnouts.provideTurnout(self.name)
        turnout.addPropertyChangeListener(self)
        self.jmri_rr_duino = jmri_rr_duino
  
    # on a property change event, first see if 
    # right type, and then write appropriate
    # value to port based on new state
    def propertyChange(self, event):
        #print "change",event.propertyName
        #print "from", event.oldValue, "to", event.newValue
        #print "source systemName", event.source.systemName
        print("transer!!")
        if not self.jmri_rr_duino.is_connected:
            return
        if (event.propertyName == "CommandedState") :
            if (event.newValue == CLOSED and event.oldValue != CLOSED) :
                print "set CLOSED for", event.source.systemName
                self.jmri_rr_duino.serversock.send((event.source.systemName+",0\r\n").encode('utf-8'))
            if (event.newValue == THROWN and event.oldValue != THROWN) :
                  print "set THROWN for", event.source.systemName
                  self.jmri_rr_duino.serversock.send((event.source.systemName+",1\r\n").encode('utf-8'))

class JMRI_RR_duino(jmri.jmrit.automat.AbstractAutomaton) :

    def __init__(self, ip_address,ip_port):
        self.ip_address=ip_address
        self.ip_port=ip_port
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #for i in range(5):
        #    Turnouttransfer(i,self)
        self.is_connected = False

    # init() is the place for your initialization
    def init(self) :
        pass

    def process_declare_objects(self,msg):
        if msg.startswith("NEW-TURNOUTS:"):
            is_turnout=True
        elif msg.startswith("NEW-SENSORS:"):
            is_turnout=False
        else:
            print "invalid message:",msg
            return
        msg = msg[msg.find(":")+1:].rstrip()  #strip out the message header

        names_list = msg.split(" ")  #get all names
        #now create the corresponding JMRI objects
        for name in names_list:
            if is_turnout:
                if not turnouts.getBySystemName("ITRT"+name):
                    print "ITRT"+name," does not exist, create it"
                    turnouts.newTurnout("ITRT"+name,"RRduino "+name)
                    #create listener
                    TurnoutTransfer(name,self)
            else:
                if not sensors.getBySystemName("ISRS"+name):
                    print "ISRS"+name," does not exist, create it"
                    sensors.newSensor("ISRS"+name,"RRduino "+name)

    # handle() is called repeatedly until it returns false.
    def handle(self) :
        if not self.is_connected:
            #not connected yet try to
            self.is_connected = True
            #try:
            self.serversock.connect((self.ip_address,self.ip_port))
            #except:
            #    print "connect error!"
            #    self.is_connected = False

        if not self.is_connected:
            return True
        
        #see if there is a msg waiting
        try:
            msg = self.serversock.recv(200).decode('utf-8')
        except:
            msg = ""
        if msg=="":
            return True
        #split messages, there might be several
        msg_list=msg.split("\n")
        for msg in msg_list:
            msg.rstrip()
            print "message is:",msg
            if msg=="":
                continue
            if msg.startswith("NEW-"):
                self.process_declare_objects(msg)
                continue
            if not msg.startswith("ISRS"):
                print "invalid message:",msg
                continue
            bus,sep,end = msg.partition(":")
            if sep == "":
                print  msg,": not valid"
                continue
            try:
                bus_int = int(bus[4:])
            except:
                print msg,": bus number not valid"
                continue
            #get node address
            add,sep,end = end.partition(":")
            if sep=="":
                print msg,": node address not valid"
            try:
                add_int = int(add)
            except:
                print msg,": node address not valid"
                continue
            subadd,sep,value = end.partition(",")
            if sep=="":
                print msg,": value not valid"
            try:
                subadd_int = int(subadd)
            except:
                print msg,": subaddress not valid"
                continue
            try:
                val_int = int(value)
            except:
                print msg,": value not valid"
                continue
            print msg,"<->",bus_int,add_int,subadd_int,val_int
            begin,sep,end=msg.partition(",")
            s = sensors.getBySystemName( begin )
            if s is None :
                print begin, " Not Available"
                return 1
            if val_int == 1 :
                s.setKnownState(ACTIVE)
            else:
                s.setKnownState(INACTIVE)

        return True    # to continue 0 to Kill Script

    def write(self, data) : 
        pass
    
    def flush(self) :
        pass


#with open("jmri_rr_duino.cfg") as cfg_file:
#    config = json.load(cfg_file)

# create one of these; provide the name of the serial port
#a = JMRI_RR_duino(config["server_ip"],config["server_jmri_port"])
a=JMRI_RR_duino("192.168.0.22",50010)

# set the thread name, so easy to cancel if needed
a.setName("JMRI_RR_duino script")

# start running
a.start()

# setup  complete
a.flush()
