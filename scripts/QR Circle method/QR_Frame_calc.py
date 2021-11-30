#!/usr/bin/env python
import numpy as np
def calcAngle(vec1,vec2):
  # calculates angle between 2 vectors
  angle = np.arctan2(vec2[1], vec2[0]) - np.arctan2(vec1[1], vec1[0])
  return angle

def QR_frame_calc(odopos, qrpos):
  #Threshold for how big error can be for fin found coordinate
  ACC_THRESHOLD = 0.7
  #Position of OR codes in respect to robot position
  odo1 = np.array(odopos[0:2])
  odo2 = np.array(odopos[2:4])
  #Position of QR Codes qr frame
  QR1 = np.array(qrpos[0:2])
  QR2 = np.array(qrpos[2:4])
  #radius of the circles in qr frame
  rQR1 = np.sqrt(QR1[0]**2 + QR1[1]**2)
  rQR2 = np.sqrt(QR2[0]**2 + QR2[1]**2)

  #Distance between QR codes
  dist = np.linalg.norm(odo1-odo2)

  #distance from radius of circle1 to intersection middle point
  b = (rQR1**2 - rQR2**2 + dist**2)/(2**dist)
  #MiddLe point coordinate
  Pmid = odo1 + (b * (odo2 - odo1))/dist
            
  #Distance from intersection to middle point
  a = np.sqrt(rQR1**2-dist**2)

  #Calculating both intersecting points
  interx1 = Pmid[0] + a*(odo2[1]-odo1[1])/dist
  interx2 = Pmid[0] - a*(odo2[1]-odo1[1])/dist

  intery1 = Pmid[1] - a*(odo2[0]-odo1[0])/dist
  intery2 = Pmid[1] + a*(odo2[0]-odo1[0])/dist

  #Angle that the frame has to be rotated calc
  QR1vec = [odo1[0] - interx1, odo1[1] - intery1]
  rot = calcAngle(odo1, QR1vec)

  #Testing rotated angle and frame shift with another other QR code info
  c = np.cos(rot)
  s = np.sin(rot)
  rotMatrix = np.array( ((c,-s), (s, c)))
  testVec = rotMatrix.dot(QR2)

  #See if point matches with odometry frame coordinate
  ACC = abs(np.array([interx1,intery1]) + testVec - odo2)
  print ("Accuracy 1:",ACC)
  if ACC[0]<ACC_THRESHOLD and ACC[1] < ACC_THRESHOLD:
    return np.array([interx1,intery1]), rot

  #Check other intersection if this does not work
  QR2vec = [odo2[0] - interx2, odo2[1] - intery2]
  tempRotAngle = calcAngle(odo2, QR2vec)

  c = np.cos(rot)
  s = np.sin(rot)
  rotMatrix = np.array(((c, -s), (s, c)))
  testvec2 = rotMatrix.dot(QR1.T)
  #If the rotation Works for the second QR, return
  ACC = abs(np.array([interx1,intery1]) + testvec2 - odo1)
  print("Accuracy 2: ",ACC)
  if ACC[0] < ACC_THRESHOLD and ACC[1] < ACC_THRESHOLD:
    return np.array([interx2,intery2]), tempRotAngle

  return False,False