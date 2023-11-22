from GUI import GUI
from HAL import HAL

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

HAL.takeoff(3)

while not ((victims_x-1 < x_pos) and (x_pos <victims_x+1) and (victims_y-1 < y_pos) and (y_pos < victims_y+1)):
  	GUI.showImage(HAL.get_frontal_image())
  	GUI.showLeftImage(HAL.get_ventral_image())
  	x_pos = HAL.get_position()[0]
  	y_pos = HAL.get_position()[1]
  	HAL.set_cmd_pos(victims_x, victims_y, 3, angle)
  	# time.sleep(0.01)
  	print('x:', x_pos)
  	print('y:', y_pos)  
	

while True:
    # Enter iterative code!
    GUI.showImage(HAL.get_frontal_image())
    GUI.showLeftImage(HAL.get_ventral_image())
