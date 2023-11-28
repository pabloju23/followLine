from GUI import GUI
from HAL import HAL
import cv2
import math 


# Origin: 40º16’48.2” N, 3º49’03.5” W = 430492E, 4459162N
# Goal: 40º16’47.23” N, 3º49’01.78” W = 430532E, 4459132N
# Easting coords are related to the x axis and Northing is related to y axis.
# Goal — Origin: 40E,30S
# Therefore, the (x, y) coordinates the drone should approach are (40, -30)
victims_x = 40
victims_y = -30

x_vel = 0.25
angle = 0.6
iterations = 0
spiral_iterations = 300
landing_margin = 0.07

x_pos = HAL.get_position()[0]
y_pos = HAL.get_position()[1]

inpos = True

# Parameters for the spiral
spiral_radius_increment = 0.1  # Increase this value to adjust the spiral spacing
spiral_angle_increment = 0.1   # Increase this value to adjust the angle between spiral points
current_spiral_radius = 0.0


HAL.takeoff(3)

while inpos:
  GUI.showImage(HAL.get_frontal_image())
  GUI.showLeftImage(HAL.get_ventral_image())
  x_pos = HAL.get_position()[0]
  y_pos = HAL.get_position()[1]
  HAL.set_cmd_pos(victims_x, victims_y, 3, angle)
  # time.sleep(0.01)
  print('x:', x_pos)
  print('y:', y_pos)  
  
  if ((victims_x-0.2 < x_pos) and (x_pos <victims_x+0.2) and (victims_y-0.2 < y_pos) and (y_pos < victims_y+0.2)):
      inpos = False
  	
while True:
  # Let's go
    GUI.showImage(HAL.get_frontal_image())
    GUI.showLeftImage(HAL.get_ventral_image())
    
    # Update the spiral position
    current_spiral_radius += 0.01
    spiral_x = victims_x + current_spiral_radius * math.cos(angle)
    spiral_y = victims_y + current_spiral_radius * math.sin(angle)
    
    # Set the drone's position
    HAL.set_cmd_pos(spiral_x, spiral_y, 3, angle)
    
    # Print the current position (for debugging)
    print('x:', spiral_x)
    print('y:', spiral_y)
    print('radius:', current_spiral_radius)
