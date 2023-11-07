from GUI import GUI
from HAL import HAL
import cv2

i = 0

while True:
    img = HAL.getImage()
    
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Define the lower and upper HSV ranges for red
    lower_red = (0, 125, 125)
    upper_red = (30, 255, 255)
    
    # Create a mask for red objects
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Apply Gaussian blur to reduce noise
    red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)
    
    # Apply morphological operations (erosion and dilation) to refine the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    red_mask = cv2.erode(red_mask, kernel, iterations=2)
    red_mask = cv2.dilate(red_mask, kernel, iterations=2)
    
    # Find contours in the binary mask
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours based on area or other criteria to remove noise
    
    if contours:
        # Calculate moments for the largest contour
        M = cv2.moments(max(contours, key=cv2.contourArea))
        if M["m00"] != 0:
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
        else:
            cX, cY = 0, 0
            
        if cX>0:
            err = 320 - cX
            HAL.setV(1)
            HAL.setW(0.01 * err)
              
        # Implement the control logic based on cX and adjust the robot's velocity
        
    else:
        cX, cY = 0, 0
        
    GUI.showImage(red_mask)
    print('%d cX: %.2f cy: %.2f' % (i, cX, cY))
    i += 1

