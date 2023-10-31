from GUI import GUI
from HAL import HAL
import cv2

i = 0
while True:
    img = HAL.getImage()
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 125, 125), (30, 255, 255))
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    M = cv2.moments(contours[0])
    
    if M["m00"] != 0:
      cX = M["m10"] / M["m00"]
      cY = M["m01"] / M["m00"]
    else:
      cX, cY = 0, 0
      
    if cX>0:
      err = 320 - cX
      HAL.setV(1)
      HAL.setW(0.01 * err)
      
    GUI.showImage(red_mask)
    print('%d cX: %.2f cy: %.2f' % (i, cX, cY))
    i = i+1