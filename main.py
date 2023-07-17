from Omen.pluto import Pluto
from Omen.pid import PID
from Omen.aruco import track_aruco
import threading
import keyboard
import time
import cv2
import numpy as np

drone = Pluto()

file = open("./Omen/logs/log.txt", "w")
file.close()

#------
def connector():

    while 1:
        if not drone.connected:
            if keyboard.is_pressed('b'):
                if not drone.connect():
                    print('Failed to Connect')
                else:
                    print('Connected')
                    time.sleep(1)
                    
                    drone.connected = 1

        else:
            if keyboard.is_pressed('n'):
                drone.disconnect()
                print('Disconnected')
                drone.connected = 0

            if keyboard.is_pressed('q'):
                print('q')
                drone.send_MSP_SET_COMMAND(drone.msp.TAKE_OFF)          

            if keyboard.is_pressed('e'):
                print('e')
                drone.send_MSP_SET_COMMAND(drone.msp.LAND)
            
            if keyboard.is_pressed('space'):
                drone.arm()
                print('Armed')   

            if keyboard.is_pressed('esc'):
                drone.disarm()
                print('Disarmed')

            drone.send_MSP_RAW_RC()
        
        time.sleep(0.08)

#----------------

def autoStabilizer():

    pidX = PID(1/2, 0, 20, 0, -50, 50)
    pidY = PID(1/2, 0, 20, 0, -50, 50)
    pidZ = PID(30, 0, 50, 0, -100, 100)
    
    while(1):

        zavg = drone.zavg
        xavg = drone.xavg
        yavg = drone.yavg
        delta = 50
        xmin = xavg-delta
        xmax = xavg+delta
        ymin = yavg-delta
        ymax = yavg+delta

        pidX.setpoint = xavg
        pidY.setpoint = yavg
        pidZ.setpoint = zavg

        defPitch = drone.defPitch
        defRoll = drone.defRoll
        defThrottle = drone.defThrottle

        frame,det,pos_x,pos_y,angle_z,size = track_aruco()
        cv2.line(img= frame, pt1=(xmin,ymin), pt2= (xmax,ymin), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
        cv2.line(img= frame, pt1=(xmin,ymin), pt2= (xmin,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
        cv2.line(img= frame, pt1=(xmin,ymax), pt2= (xmax,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
        cv2.line(img= frame, pt1=(xmax,ymin), pt2= (xmax,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)

        points = np.array([[540,200],[540,380],[190,380],[190,200]])
        cv2.polylines(frame, [points], 1, (255,255,255))

        cv2.circle(frame,(xavg,yavg),2,(0,255),2)

        if keyboard.is_pressed('z'):
            drone.physics.rcYaw = 1300
        elif keyboard.is_pressed('x'):
            drone.physics.rcYaw = 1700
        elif det:
            if angle_z>5:
                drone.physics.rcYaw = 1300
            elif angle_z<-5:
                drone.physics.rcYaw = 1700
            else:
                drone.physics.rcYaw = 1500
        else:            
            drone.physics.rcYaw = 1500

        if keyboard.is_pressed('p'):
            drone.physics.rcThrottle = 1700
        elif keyboard.is_pressed('l'):
            drone.physics.rcThrottle = 1450
        elif det:
            drone.physics.rcThrottle = defThrottle - pidZ.update(size)

        if keyboard.is_pressed('a'):
            drone.physics.rcRoll = defRoll - 200 
        elif keyboard.is_pressed('d'):
            drone.physics.rcRoll = defRoll + 200
        elif det:
            drone.physics.rcRoll = defRoll - pidX.update(pos_x)

        if keyboard.is_pressed('s'):
            drone.physics.rcPitch = defPitch - 200
        elif keyboard.is_pressed('w'):
            drone.physics.rcPitch = defPitch + 200
        elif det:
            drone.physics.rcPitch = defPitch + pidY.update(pos_y)

        drone.send_MSP_RAW_RC()

        file = open("./Omen/logs/log.txt", "a")

        file.write("PosX == %s" % (pos_x))
        file.write("\tPosY == %s" % (pos_y))
        file.write("\tSize == %s" % (size))
        file.write("\tAngle == %s" % (angle_z))
        file.write("\n")
        file.write("\tdefRoll == %s" % (defRoll))
        file.write("\tdefPitch == %s" % (defPitch))
        file.write("\trcThrottle == %s" % (drone.physics.rcThrottle))
        file.write("\trcYaw == %s" % (drone.physics.rcYaw))
        file.write("\trcRoll == %s" % (drone.physics.rcRoll))
        file.write("\trcPitch == %s" % (drone.physics.rcPitch))
        file.write("\n\n")

        file.close()


        cv2.putText(frame,"Roll="+str(drone.physics.rcRoll),(10,10),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
        cv2.putText(frame,"Pitch="+str(drone.physics.rcRoll),(10,10+15),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
        cv2.putText(frame,"Throttle="+str(drone.physics.rcThrottle),(10,10+30),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
        cv2.putText(frame,"Yaw="+str(drone.physics.rcYaw),(10,10+45),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)

        cv2.putText(frame,"defRoll="+str(drone.defRoll),(10,10+60),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
        cv2.putText(frame,"defPitch="+str(drone.defPitch),(10,10+75),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)

        key = cv2.waitKey(3) & 0xFF
        
        cv2.imshow('frame', frame)

        time.sleep(0.005)

    cv2.destroyAllWindows()

def autopilot():
    time.sleep(5)
    drone.send_MSP_SET_COMMAND(drone.msp.TAKE_OFF)          
    time.sleep(5)
    drone.send_MSP_SET_COMMAND(drone.msp.LAND)   
    time.sleep(5)
    drone.send_MSP_SET_COMMAND(drone.msp.TAKE_OFF)          
    time.sleep(5)
    drone.send_MSP_SET_COMMAND(drone.msp.LAND)   

    time.sleep(5)
    drone.send_MSP_SET_COMMAND(drone.msp.TAKE_OFF)  
    time.sleep(10) 

    drone.xavg = 540
    drone.yavg = 200

    time.sleep(15)

    for i in range(200,380):
        drone.yavg = i
        time.sleep(0.025)
    
    time.sleep(3)

    for i in range(540,190,-1):
        drone.xavg = i
        time.sleep(0.025)
    
    time.sleep(3)

    for i in range(380,200,-1):
        drone.yavg = i
        time.sleep(0.025)
    
    time.sleep(3)

    for i in range(190,540):
        drone.xavg = i
        time.sleep(0.025)
    
    time.sleep(5)

    drone.send_MSP_SET_COMMAND(drone.msp.LAND)     

# if not drone.connect():
#     print('Failed to Connect')
#     exit(0)
# else:
#     print('Connected')

# --
# threads
t_transmitter = threading.Thread(target=drone.msp.transmit)
t_connector = threading.Thread(target=connector)
t_autoStablizer = threading.Thread(target=autoStabilizer)
t_autopilot = threading.Thread(target=autopilot)


t_connector.start()
print('Connector started')

t_transmitter.start()
print('Transmitter started')

# t_autoStablizer.start()
# print('Auto Stablizer started')

# t_autopilot.start()
# print('Auto Pilot started')



# t_keyboard.join()
# print('Keyboard stopped')

# t_transmitter.join()
# print('Transmitter stopped')

# t_autoStablizer.join()
# print('Auto Stablizer stopped')

# t_autopilot.join()
# print('Auto Pilot stopped')


# drone.msp.stopEvent.set()
# print('Trans ended')

# drone.disconnect()
# print('disconnected')