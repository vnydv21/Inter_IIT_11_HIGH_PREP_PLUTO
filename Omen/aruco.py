import numpy as np
import cv2
import cv2.aruco as aruco
import math

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_EXPOSURE,0)

MAT_COEFF = np.array([[900,0,600],[0,900,400],[0,0,1]],dtype='float32') # Camera instrinsic parameters Matrix
DST_COEFF = np.array([0,0,0,0],dtype='float32') # Distortion Coefficient Matrix

def track_aruco():
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)  # Use 4x4 Aruco Code Dictionary
        parameters = aruco.DetectorParameters()  # Marker detection parameters
        detector = aruco.ArucoDetector(aruco_dict, parameters)   # Detector to detect Aruco Codes
        corners, ids, rejected_img_points = detector.detectMarkers(gray) # list of ids and the corners beloning to each id

        res = {
          "frame":None,
          "ids":[],
          "data":[]
        }

        if np.all(ids is not None):  # If markers are detected
            for i in range(len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, MAT_COEFF, DST_COEFF)
                (rvec - tvec).any()
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                cv2.drawFrameAxes(frame, MAT_COEFF, DST_COEFF, rvec, tvec, 0.01)  # Draw Axis

                # Center Calculation
                tx = (corners[i][0][0][0]+corners[i][0][1][0]+corners[i][0][2][0]+corners[i][0][3][0])/4
                ty = (corners[i][0][0][1]+corners[i][0][1][1]+corners[i][0][2][1]+corners[i][0][3][1])/4
                
                # Midline Calculation
                l1x = int((corners[i][0][0][0]+corners[i][0][3][0])/2)
                l1y = int((corners[i][0][0][1]+corners[i][0][3][1])/2)
                l2x = int((corners[i][0][1][0]+corners[i][0][2][0])/2)
                l2y = int((corners[i][0][1][1]+corners[i][0][2][1])/2)

                # Angle Calculation
                if l1x==l2x:
                  if l1y<l2y:
                    angle_z = 90
                  else:
                    angle_z = 270
                else:
                  angle_z = math.degrees(math.atan((l2y-l1y)/(l2x-l1x)))
                if l2x<l1x:
                  angle_z += 180
                elif l2x>l1x and l2y<l1y:
                  angle_z += 360
                if angle_z>180:
                  angle_z-=360

                # Height Calculation
                size = math.sqrt((l1x-l2x)**2+(l1y-l2y)**2)
                
                # Draw and write
                # cv2.line(frame,(l1x,l1y),(l2x,l2y),(255,0,0),2)
                cv2.circle(frame,(int(tx),int(ty)),5,(255,255,255),1)
                cv2.putText(frame,"X="+str(int(tx)) + "  Y=" + str(int(ty)),(int(tx)+20,int(ty)),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
                cv2.putText(frame,"Ang="+str(int(angle_z)),(int(tx)+20,int(ty)+15),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
                cv2.putText(frame,"Z="+str(int(size)),(int(tx)+20,int(ty)+30),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)
                cv2.putText(frame,"ID="+str(ids[i][0]),(int(tx)+20,int(ty)+45),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=1)

                res["ids"].append(ids[i][0])
                res["data"].append((tx,ty,angle_z,size))
        
        res["frame"] = frame
        
        return res