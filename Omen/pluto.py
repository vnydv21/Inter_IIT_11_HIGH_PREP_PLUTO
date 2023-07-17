import socket
import time
import struct
from Omen.msp import MSP, Packet
import json

import Omen.logger as logger

log = logger.GetLogger("Pluto", "Omen/logs/Pluto.log")

class Physics:

    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.rssi = 0
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0
        self.magX = 0
        self.magY = 0
        self.magZ = 0
        self.alt = 0
        self.trim_roll = 0
        self.trim_pitch = 0

        self.rcThrottle = 1500
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.rcYaw = 1500
        self.rcAUX1 = 1500
        self.rcAUX2 = 1500
        self.rcAUX3 = 1500
        self.rcAUX4 = 1500


class Pluto:

    def __init__(self, zavg, xavg, yavg, defPitch, defRoll, defThrottle, name = "plutoAlpha"):
        self.name = name
        self.server = None
        self.msp = MSP(self.server)        
        self.physics = Physics()
        self.calibrate = 0
        self.connected = 0
        self.tookoff = 0

        file = open('./Omen/callibration_parameters.txt','r')
        cal = json.loads(file.read())
        file.close()

        self.zavg = zavg
        self.xavg = xavg
        self.yavg = yavg
        self.defPitch = defPitch
        self.defRoll = defRoll
        self.defThrottle = defThrottle

    def connect(self, host="192.168.4.1",port=23):     
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.settimeout(0.1)
            self.msp.server = self.server
            log.debug(f"Connecting with {host}({port})")
            print("Attempting to connect...")
            self.server.connect((host, port))
            log.info("Connected.")

            self.server.sendall(b"+++AT")
            log.debug("sent: +++AT")
            
            data = self.server.recv(1024)
            
            if data == b"OK\r\n":
                log.info("rcvd: OK")
                self.connected = 1
                self.disarm()
                return True


            else:
                log.error("NO REPLY")
    
        except TimeoutError:
            log.error("Failed To Connect: Connection TIME_OUT")

        except:
            log.error("x Failed To Connect: NO NETWORK")

        return False            

    def disconnect(self):
        self.connected=0
        self.server.close()
        self.server = None
        self.msp.server = self.server
        log.info("Disconnected.")

    def takeoff(self):
        if self.tookoff==0:
            self.send_MSP_SET_COMMAND(self.msp.TAKE_OFF)
            self.tookoff=1

    def land(self):
        if self.tookoff==1:
            self.send_MSP_SET_COMMAND(self.msp.LAND)
            self.tookoff=0

    def send_MSP_RAW_RC(self):
        log.info('sending MSP_RAW_RC')
        mspCmd = MSP.MSP_SET_RAW_RC
        logifOK = 'sent MSP_RAW_RC'
        logifNot = 'Failed to send MSP_RAW_RC'

        data = [             
            self.physics.rcRoll,
            self.physics.rcPitch,
            self.physics.rcThrottle,
            self.physics.rcYaw,
            self.physics.rcAUX1,
            self.physics.rcAUX2,
            self.physics.rcAUX3,
            self.physics.rcAUX4
            ]

        # check struct.pack format on python docs
        # H -> uint_16 - 2 byte
        # 8H -> 8 * uint_16 elements
        data_length = len(data)*2
        data_format = "8H"

        packet = Packet(True, mspCmd, data_length, data, data_format)

        #self.msp.send(packet)
        self.msp.SEND_QUEUE.put(packet.byteStream)


    def send_MSP_SET_COMMAND(self, cmd):
        log.info('sending MSP_SET_COMMAND')
        mspCmd = MSP.MSP_SET_COMMAND

        logifOK = 'sent MSP_SET_COMMAND'
        logifNot = 'Failed to send MSP_SET_COMMAND'

        data = [cmd]

        data_length = len(data)*2
        data_format = "1H"

        packet = Packet(True, mspCmd, data_length, data, data_format)

        #self.msp.send(packet)
        self.msp.SEND_QUEUE.put(packet.byteStream)        

    def send_MSP_SET_ACC_TRIM(self, usePhysics : bool, trimRoll : int = 0, trimPitch : int = 0):
        '''usePhysics : use already set values,
        else set new values
        '''

        if not usePhysics:
            self.physics.trim_roll = trimRoll
            self.physics.trim_pitch = trimPitch

        log.info('sending MSP_SET_ACC_TRIM')        
        mspCmd = MSP.MSP_SET_COMMAND

        data = [self.physics.trim_pitch, self.physics.trim_roll]

        data_length = len(data)*2
        data_format = "2H"

        packet = Packet(True, mspCmd, data_length, data, data_format)

        #self.msp.send(packet)
        self.msp.SEND_QUEUE.put(packet.byteStream)        


    def arm(self):
        ''' also send trim values '''
        self.physics.rcAUX4 = self.msp.ARM_DRONE
        self.send_MSP_RAW_RC()

        # also send init values,
        self.send_MSP_SET_ACC_TRIM(True)

            
    def disarm(self):
        self.physics.rcAUX4 = self.msp.DISARM_DRONE
        self.send_MSP_RAW_RC()

    def send_MSP_REQUEST(self, type=0):
        if type==0:
            log.info('sending MSP_ATTITUDE')
            mspCmd = MSP.MSP_ATTITUDE
        elif type==1:
            log.info('sending MSP_ALTITUDE')
            mspCmd = MSP.MSP_ALTITUDE
        elif type==2:
            log.info('sending MSP_RAW_IMU')
            mspCmd = MSP.MSP_RAW_IMU
            
        data = []
        data_length = len(data)*2
        data_format = ""
        packet = Packet(True, mspCmd, data_length, data, data_format)

        self.msp.SEND_QUEUE.put(packet.byteStream)

    def MSP_REQUEST_thread(self):
        while (1):
            # self.send_MSP_REQUEST(0)
            # self.send_MSP_REQUEST(1)
            self.send_MSP_REQUEST(2)

            while len(self.msp.RECV_QUEUE)>1:
                i = self.msp.RECV_QUEUE.popleft()
                i = i[2:]

                if len(i)>1:
                    cmd = struct.unpack("B",i[1:2])[0]
                    data = None

                    try:
                        if cmd == MSP.MSP_ATTITUDE:
                            data = struct.unpack("<2B3hB",i)
                            print("Roll=",data[2])
                            print("Pitch=",data[3])
                            print("Yaw=",data[4])
                        elif cmd == MSP.MSP_ALTITUDE:
                            data = struct.unpack("<2BihB",i)
                            print("Alt=",data[2])
                            print("Vel=",data[3])
                        elif cmd == MSP.MSP_RAW_IMU:
                            data = struct.unpack("<2B9hB",i)
                            print("AccX=",data[2])
                            print("AccY=",data[3])
                            print("AccZ=",data[4])
                            print("GyroX=",data[5])
                            print("GyroY=",data[6])
                            print("GyroZ=",data[7])
                            print("MagX=",data[8])
                            print("MagY=",data[9])
                            print("MagZ=",data[10])
                    
                    except:
                        pass


            time.sleep(0.2)
