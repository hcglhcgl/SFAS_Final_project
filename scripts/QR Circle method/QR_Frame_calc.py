#!/usr/bin/env python
import numpy as np

  #calculate an angle from vector 1 to vector 2
def calcVecAngle(vec1,vec2):
  #angleatan2 (vector2.y, vector2.x) atan2 (vectorl.y, vectorl.x)
  angle = np.arctan2(vec2[1], vec2[0]) - np.arctan2(vec1[1], vec1[0])
  if angle < 0:
    angle2 = 2 * np.pi + angle
  return angle

def QR_frame_calc(baseFrameQrPos, newFrameQrPos):
  #Threshold that defines how accurate the final solution has to be
  ACC_THRESHOLD = 0.7
  #Position of OR codes in base frame
  bfQR1 = np.array(baseFrameQrPos[0:2])
  bfQR2 = np.array(baseFrameQrPos[2:4])
  #Fosition of QR Codes in their frame
  nfQR1 = np.array(newFrameQrPos[0:2])
  nfQR2 = np.array(newFrameQrPos[2:4])
  #radius of the circles correspanding to the possible positions of QR code frane
  rQR1 = np.sqrt(nfQR1[0]**2 + nfQR1[1]**2)
  rQR2 = np.sqrt(nfQR2[0]**2 + nfQR2[1]**2)
  #Distance between QR codes
  dist = np.linalg.norm(bfQR1-bfQR2)
  #Check if circles intersect
  if dist > rQR1 + rQR2 or dist == 0 or dist < abs(rQR1-rQR2):
    return -1
  #distance to the middle point between the intersections
  midPointDist = (rQR1**2 - rQR2**2 + dist**2)/(2**dist)
  #MiddLe point position
  midPoint = bfQR1 + (midPointDist * (bfQR2 - bfQR1))/dist
  #If there is only one intersection
  if dist > rQR1 + rQR2:
    #Middle point becomes the intersection point and the center of QR frane
    framePoint = midPoint
    #Calculate vector for QR frame to first QR code
    QR1vec = [bfQR1[0] - framePoint[0],bfQR1[1]-framePoint[1]]
    #Calculate an angle that the frane has to be rotated
    rotAngle = calcVecAngle(nfQR1,QR1vec)
    return framePoint, rotAngle
  #Otherwise, there are 2 intersections
            
  #Distance from middle to intersection
  midToIntersDist = np.sqrt(rQR1**2-midPointDist**2)
  #Calc 2 intersections one of then is the frame point
  framePoint1 = midPoint + (midToIntersDist * (np.flip(bfQR2,0) - np.flip(bfQR1, 0)).dot([[1, 0],[0, -1]]))/dist
  framePoint2 = midPoint + (midToIntersDist * (np.flip(bfQR2,0) - np.flip(bfQR1, 0)).dot([[-1, 0],[0, 1]]))/dist
  #CalculLate an angle that the frame has to be rotated
  tempQR1vec = [bfQR1[0] - framePoint1[0], bfQR1[1] - framePoint1[1]]
  tempRotAngle = calcVecAngle(nfQR1, tempQR1vec)
  #Calculate rotation matrix to check if the angle workS for the other QR position
  c,s = np.cos(tempRotAngle), np.sin(tempRotAngle)
  rotMatrix = np.array( ((c,-s), (s, c)))
  testVec = rotMatrix.dot(nfQR2.T)
  #If the rotation works for the other QR, return
  accuracy = abs(framePoint1 + testVec - bfQR2)
  print ("Accuracy 1:",accuracy)
  if accuracy[0]<ACC_THRESHOLD and accuracy[1] < ACC_THRESHOLD:
    return framePoint1, tempRotAngle
  #Else check the other intersection
  tempQR1vec = [bfQR1[0] - framePoint2[0], bfQR1[1] - framePoint2[1]]
  tempRotAngle = calcVecAngle(nfQR1, tempQR1vec)
  #Calculate rotation matrix to check if the angle works for the other OR position
  c,s = np.cos(tempRotAngle) , np.sin(tempRotAngle)
  rotMatrix = np.array(((c, -s), (s, c)))
  testvec = rotMatrix.dot(nfQR2.T)
  #If the rotation Works for the other QR, return
  accuracy = abs(framePoint2 + testVec - bfQR2)
  print("Accuracy 2: ",accuracy)
  if accuracy[0] < ACC_THRESHOLD and accuracy[1] < ACC_THRESHOLD:
    return framePoint1, tempRotAngle
  #else, there is a dumpster fire somehere
  return -2