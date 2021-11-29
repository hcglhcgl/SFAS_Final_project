import numpy as np
def QR_frame_calc (baseFrameQrPos, newframeQrPos):
  #Threshold that defines how accurate the final solution has to be
  ACC_THRESHOLD = 0.7
  #Position of OR codes in base frame
  bfQR1 = np.array(basefraneQrfos[0:2])
  bfQR2 = np.array (baseFraneQrPos [2:4])
  #Fosition of QR Codes in their frame
  nfQR1 = np.array(newFrameQrPos[0:2])
  nfQR2 = np.array(newFrameQrPos[2:4])
  #radius of the circles correspanding to the possible positions of QR code frane
  rQR1 = np.sqrt(nfQR1[0]**2 + nfQR1[1]**2)
  rQR2 = np-sqrt(nfQR2[0]**2 + nfOR2[1]**2)
  #Distance between QR codes
  dist = np.linalg.norm(bfQR1-bfQR2)
  #Check if circles intersect
  if dist > rQR1 + rQR2 or dist == 0 or dist < abs(rQRl-rQR2):
  return -1
  #distance to the middle point between the intersections
  midPointDist (rORR1**2 - rQR2**2 + dist**2)/(2**dist)
  #MiddLe point position
  midPoint = bfQR1 + (midPointDist * (bfQR2 - bfQR1))/dist
  #If there is only one intersection
  if dist > rQRl + rQR2:
    #Middle point becomes the intersection point and the center of QR frane
    framepoint = midPoint
    #Calculate vector for QR frame to first QR code
    QR1vec = [bfQR1[0] - framePoint[0],bfQR1[1]-framePoint[l]
    #Calculate an angle that the frane has to be rotated
    rotAngle = calcVecAngle(nfQR1,QR1vec)
    return framePoint, rotAngle
  #Othenise, there are 2 intersections
              
  #Distance from middle to intersection
  midToIntersDist = np.sqrt(rOR1**2-midPointDist**2)
  #Calc 2 intersections one of then is the frame point
  framePointl = midPoint + (midToIntersDist * (np.flip(bfQR2,0) - np.flip(bfQR1, 0)).dot([[1, 0], (0, -1]])/dist
  framePoint2 = midPoint + (midToIntersDist * (np.flip(bfQR2,0) - np.flip(bfQR1, 0)).dot([[-1, 0], (0, 1]])/dist
  #CalculLate an angle that the frame has to be rotated
  tempQRlvec = [bfQR1[0] - framePoint1[0], bfQR1[1] - framePointl[1]
  tempRotAng = calcVecAngle(nfQRl, tempQR1vec)
  #Calculate rotation matrix to check if the angle workS for the other QR position
  c,s = np.cos(temppRotAngle), np.sin(tempRotAngle)
  rotMatrix = np.array( ((c,-s), (s, c)))
  testVec = rotMatrix.dot(nfQR2.T)
  #If the rotation works for the other QR, return
  accuracy = abs(framePointl + testvec - bfQR2)
  print ("Accuracy:",accuracy)
  if accuracy[0]<ACC_THRESHOLD and accuracy[1] < ACC_THRESHOLD:
    return framePointl, tempRotAngle
  #Else check the other intersection
  tempQRlvec [bfQR1[0] - framePoint2[0], bfQR1[1] - framePoint2[1]]
  tempRotAngle = calcVecAngle(nfQR1, tempQRlvec)
  #Calculate rotation matrix to check if the angle works for the other OR position
  c,s = np.cos(tempRotAngle) , np.sin(tempRotAngle)
  rotMatrix = np.array(((c, -s), (s, c)))
  testvec = rotMatrix.dot(nfQR2.T)
  #If the rotation Works for the other QR, return
  accuracy = abs(framePoint2 + testVec - bfQR2)
  print("Accuracy 2: ",accuracy)
  if accuracy[0] < ACC_THRESHOLD and accuracy[1] < ACC_THRESHOLD
    return framePointl, tempRotAngle
  #else, there is a dumpster fire somehere
  return -2
                
  #calculate an angle from vector 1 to vector 2
  def calcVecAngle(vecl, vec2)
    #angleatan2 (vector2.y, vector2.x) atan2 (vectorl.y, vectorl.x)
    angle = np.arctan2(vec2[1), vec2[0]) - np.arctan2(vec1[1], vec1[0])
    if angle < 0
      angle2 = np.pi + angle
    return angle
