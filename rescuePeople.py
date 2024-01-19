from GUI import GUI
from HAL import HAL
import cv2 as cv
import math 
  

# Origin: 40º16’48.2” N, 3º49’03.5” W = 430492E, 4459162N
# Goal: 40º16’47.23” N, 3º49’01.78” W = 430532E, 4459132N
# Easting coords are related to the x axis and Northing is related to y axis.
# Goal — Origin: 40E,30S
# Therefore, the (x, y) coordinates the drone should approach are (40, -30)
victims_x = 40
victims_y = -30
boat_x = 0
boat_y = 0

angle = 0.6
iterations = 0
spiral_iterations = 300
landing_margin = 0.07
height = 3

pos_th = 0.5

inpos = False  # is drone in position to rescue?
searching = False  # if drone is in position then is ready to search
return_to_base = False  # if all people are rescued then is time to base

# Parameters for the spiral
distance = 0 # Meters
distance_inc = 0.75 # Meters
spiral_angle = 0 # rads
search_max_distance = 50 # Meters
spiral_angle_increment = 0.15  # radians   
distance_th = 4.5 # threshold between victims

face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')

n_victims = 5
victims_locations = []
saved_victims = 0


def FaceFound(face, saved_victims):
  drone_location = HAL.get_position()
  dron_orientation = HAL.get_orientation()
  # Calculate the victim location from the drone orientation and victim pixel coordinates
  victim_location = (drone_location[0], drone_location[1])
  # Check if the victim is already saved
  for known_victim in victims_locations:
    # Calculate the Euclidean distance to the known victim
    euclidean_distance = math.sqrt((known_victim[0] - victim_location[0])**2 + (known_victim[1] - victim_location[1])**2)
    if euclidean_distance < distance_th:
      # The victim was already found
      return saved_victims
    
  # If we end the loop, it means that the victim has not been saved yet
  victims_locations.append(victim_location)  # store the victim location
  saved_victims += 1
  print('saved victim at location: ', victim_location)
  print('saved victims: ', saved_victims)
  return saved_victims
  
  
HAL.takeoff(3)

while not inpos:
  GUI.showImage(HAL.get_frontal_image())
  GUI.showLeftImage(HAL.get_ventral_image())
  x_pos = HAL.get_position()[0]
  y_pos = HAL.get_position()[1]
  HAL.set_cmd_pos(victims_x, victims_y, height, angle)
  
  if ((victims_x-pos_th< x_pos) and (x_pos <victims_x+pos_th) and (victims_y-pos_th < y_pos) and (y_pos < victims_y+pos_th)):
      inpos = True
      searching = True
      print('drone in position to rescue people')
  	
while searching:
  # Get Cameras data
  ventral_img = HAL.get_ventral_image()
  frontal_img = HAL.get_frontal_image()
  GUI.showImage(HAL.get_frontal_image())
  GUI.showLeftImage(HAL.get_ventral_image())
  x_pos = HAL.get_position()[0]
  y_pos = HAL.get_position()[1]
  
  HAL.set_cmd_pos(victims_x, victims_y, height, angle)
  # Transform the image to grayscale
  img_gray = cv.cvtColor(ventral_img, cv.COLOR_BGR2GRAY)
  # Check for faces
  for im_angle in range (0, 365, 10):
    # Compute rotation matrix
    (h, w) = img_gray.shape[:2]
    center = (w // 2, h // 2)
    M = cv.getRotationMatrix2D(center, im_angle, 1.0)
    # Perform the rotation
    im_rot = cv.warpAffine(img_gray, M, (w, h))
    # Detect faces
    detected_faces = face_cascade.detectMultiScale(im_rot, 1.1, 4)
    if(len(detected_faces) > 0):
      for face in detected_faces:
        saved_victims = FaceFound(face, saved_victims)
  # Increment spiral angle
  spiral_angle += spiral_angle_increment
  # Increment spiral distance
  distance = (spiral_angle/(math.pi*2)) * distance_inc # For every loop increment the distance
  # Calculate new target location
  victims_x = victims_x + distance * math.cos(spiral_angle)
  victims_y = victims_y + distance * math.sin(spiral_angle)
  
  if saved_victims == n_victims:
    searching = False
    return_to_base = True
    print('Todas las víctimas han sido rescatadas, volviendo al bote')
    
while return_to_base:
  HAL.set_cmd_pos(boat_x, boat_y, height, angle)
  # Check if drone is in postion
  x_pos = HAL.get_position()[0]
  y_pos = HAL.get_position()[1]
  euclidean_distance = math.sqrt((boat_x - x_pos)**2 + (boat_y - y_pos)**2)
  ready_to_land = euclidean_distance < 1
  if ready_to_land:
    print('aterrizando')
    HAL.land()
    if HAL.get_landed_state() == 1:
      print('aterrizado!!')
  
while True:
  # Let's go
