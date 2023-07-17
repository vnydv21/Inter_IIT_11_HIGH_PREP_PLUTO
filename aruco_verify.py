from Omen.aruco import track_aruco
import cv2
import time
import numpy as np

xavg=374
yavg=175
delta=50
xmin = xavg-delta
xmax = xavg+delta
ymin = yavg-delta
ymax = yavg+delta

file = open("./log.txt", "w")
file.close


while(1):
  aruco = track_aruco()
  frame = aruco["frame"]
  cv2.line(img= frame, pt1=(xmin,ymin), pt2= (xmax,ymin), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
  cv2.line(img= frame, pt1=(xmin,ymin), pt2= (xmin,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
  cv2.line(img= frame, pt1=(xmin,ymax), pt2= (xmax,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
  cv2.line(img= frame, pt1=(xmax,ymin), pt2= (xmax,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)

  points = np.array([[540,200],[540,380],[190,380],[190,200]])
  cv2.polylines(frame, [points], 1, (255,255,255))

  cv2.imshow('frame', frame)

  key = cv2.waitKey(3) & 0xFF

  time.sleep(0.01)
  # file = open("./log.txt", "a")
  # file.write("Size %s" % (size))
  # file.write("X" + str(pos_x))
  # file.write("Y" + str(pos_y))
  # file.write("A" + str(angle_z))
  # file.write("\n")
  # file.close()

  size=10
  print(np.clip(int((size-14)*100),-100,100))
