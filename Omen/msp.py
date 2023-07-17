from queue import Queue
import struct
import Omen.logger as logger
from threading import Event
import time
from collections import deque

log = logger.GetLogger("MSP", "Omen/logs/MSP.log")

class Packet:

    def __init__(self,isSend, mspCmd, data_length, data, data_format):        
        checksum = 0

        packet = []
        packet.extend(MSP.PKT_HEAD)
        if isSend:
            packet.append(MSP.PKT_IN)
        else:
            packet.append(MSP.PKT_OUT)


        packet.append(data_length)
        packet.append(mspCmd)
        packet.extend(data)
        
        # convert to bytes and Checksum        
        for i in struct.pack('<2B'+data_format,*packet[3:]):
            checksum = checksum ^ i
        packet.append(checksum)
        
        # convert to bytes
        self.byteStream = struct.pack('<5B'+data_format+'B',*packet)
    



class MSP:

    # the MSP commands
    MSP_FC_VERSION=3
    MSP_RAW_IMU=102
    MSP_RC = 105
    MSP_ATTITUDE=108
    MSP_ALTITUDE=109
    MSP_ANALOG=110
    MSP_SET_RAW_RC=200
    MSP_ACC_CALIBRATION=205
    MSP_MAG_CALIBRATION=206
    MSP_SET_MOTOR=214
    MSP_SET_ACC_TRIM=239
    MSP_ACC_TRIM=240
    MSP_EEPROM_WRITE = 250
    MSP_SET_POS= 216
    MSP_SET_COMMAND = 217

    # check manual for commands
    MAGHOLD_ON = 1200
    HEADFREE_MODE_ON = 1500
    DEV_MODE_ON = 1500
    ALT_HOLD_ON = 1500
    THROTTLE_MODE_ON = 2000
    ARM_DRONE = 1500
    DISARM_DRONE = 2100

    PKT_HEAD = [0x24,0x4d] #$M
    PKT_IN = 0x3c # <
    PKT_OUT = 0x3e # >

    # do commands
    NONE_COMMNAD = 0
    TAKE_OFF = 1
    LAND = 2


    def __init__(self,server):
        # the "drone"
        self.server = server
        self.SEND_QUEUE = Queue()
        self.RECV_QUEUE = deque()
        self.stopEvent = Event()

    
    def send(self, packet):
        self.SEND_QUEUE.put(packet.byteStream)


    def transmit(self):
        '''call this using threads'''

        log.info("Transmission Started")

        while not self.stopEvent.is_set():
            try:
                if not self.SEND_QUEUE.empty():
                    self.server.sendall(self.SEND_QUEUE.get())
                    log.info(f"Queue len {self.SEND_QUEUE.qsize()}")
            except:
                pass


            time.sleep(0.004)

        log.info("Transmission Ended")


    def receive(self):
        
        while not self.stopEvent.is_set():
            # mspCmd = MSP.MSP_ATTITUDE
            # data =[]
            # data_format = ""
            # data_length = len(data)*2
            # packet = Packet(True, mspCmd, data_length, data, data_format)

            # self.server.sendall(packet.byteStream)
            bytestream = self.server.recv(1024)
            packets = bytestream.split(b'$')
            for i in packets:
                if len(i)>1:
                    if i[0:2] == b'M>':
                        self.RECV_QUEUE.append(i)
                    else:
                        if len(self.RECV_QUEUE)>0:
                            x = self.RECV_QUEUE.pop()
                            i=x+i
                        self.RECV_QUEUE.append(i)


            # print(bytestream)
            # print(bytestream)
            # print("")
            
                    
                    # if data!=None:
                    #     print(data)


            # print("................")
            # try:
            #     data = struct.unpack("<3BH3hB",bytestream)
            #     print(data)
            # except:
            #     pass

            time.sleep(0.05)
