import serial,sys

class serial_bus:
    def __init__(self,port,baudrate):
        self.ser_port = serial.Serial()
        self.ser_port.port = port
        self.ser_port.baudrate = baudrate
        self.ser_port.timeout=0
        self.ser_port.write_timeout = 0
        self.to_send=b""
        self.to_send_pos=0
        self.rcv_buffer = b""

    def start(self):
        if not self.ser_port.is_open:
            self.ser_port.open()
    def stop(self):
        if self.ser_port.is_open:
            self.ser_port.close()

    def send(self,msg): #msg must be bytes array
        if len(self.to_send)>0:
            print("overrun of the sending buffer")
        self.to_send=msg

    def sending(self):
        return len(self.to_send)>0

    def read(self):
        if len(self.rcv_buffer)>0:
            res = self.rcv_buffer
            self.rcv_buffer = b""
            return res
        else:
            return None

    def available(self):
        return len(self.rcv_buffer)
    
    def process_IO(self):
        if self.to_send:  #still sending
            if self.to_send_pos < len(self.to_send):
                #print("sending msg=",self.to_send[self.to_send_pos:])
                try:
                    nb = self.ser_port.write(self.to_send[self.to_send_pos:])
                    self.to_send_pos += nb
                except BaseException:
                    pass
            else:
                self.to_send = b""
                self.to_send_pos = 0
                #print("sending is done")
        else:   #see if we have received something
            try:
                self.rcv_buffer +=  self.ser_port.read()
            except BaseException:
                print("exception while read serial")
