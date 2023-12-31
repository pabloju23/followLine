from GUI import GUI
from HAL import HAL
import cv2
import time

# Parámetros del controlador PID
kp = 0.002
ki = 0.0000
kd = 0.00058

# Inicialización de variables del PID
last_error = 0
integral = 0
last_time = time.time()

# Guarda las coordenadas iniciales
start_time = time.time()
start_coordinates = None
completion_time_threshold = 10  # Tiempo mínimo para considerar que el coche ha completado el circuito
x_margin = 10  # Margen en la coordenada x
y_margin= 0

i = 0
v = 6
curve_speed_factor = 0.2
min_speed = 3
initial_speed = 7


def calculate_speed_factor(curve_angle):
  if curve_angle > 5:
    return 1 - 0.01 * abs(curve_angle)
  else:
    return 1


while True:
    img = HAL.getImage()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = (0, 125, 125)
    upper_red = (30, 255, 255)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)

    red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    red_mask = cv2.erode(red_mask, kernel, iterations=2)
    red_mask = cv2.dilate(red_mask, kernel, iterations=2)

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)

        if area > 100:  # Filtrar contornos pequeños como ruido
            M = cv2.moments(max_contour)
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
            err = 320 - cX

            # Calcula el tiempo que ha pasado
            current_time = time.time()
            dt = current_time - last_time

            # Parte proporcional
            proportional = kp * err

            # Parte integral
            integral += ki * err * dt

            # Parte derivativa
            derivative = kd * (err - last_error) / dt

            # Calcula la salida del PID
            output = proportional + integral + derivative

            # Calcula el ángulo de la curva
            ellipse = cv2.fitEllipse(max_contour)
            curve_angle = ellipse[2]

            # Ajusta la velocidad en función del ángulo de la curva
            speed_factor = calculate_speed_factor(curve_angle)
            print('speed_factor: ', speed_factor) #0.68
            print('output: ', output) #0.21
            # Calcula la velocidad considerando el límite inferior y la velocidad inicial
            speed = max(min_speed, initial_speed * speed_factor * (1-0.01*abs(output)))

            #speed = v
            # Ajustar la velocidad y el ángulo de dirección en función de la salida del PID y la velocidad
            HAL.setV(speed)
            print('v_lineal: ', speed)
            HAL.setW(output)

            # Registra las coordenadas iniciales si aún no se han registrado
            if start_coordinates is None:
                start_coordinates = (cX, cY)  # Coordenadas iniciales

                # Verifica si el coche ha pasado por las coordenadas iniciales después de un cierto tiempo
            if start_coordinates is not None and current_time - start_time > completion_time_threshold:
                if abs(cY - start_coordinates[1]) < y_margin and abs(cX - start_coordinates[0]) < x_margin:
                    print("Circuito completado!")
                    HAL.setV(0)  # Detén el coche
                    HAL.setW(0)
                    #break

            # Actualiza las variables del PID para la próxima iteración
            last_error = err
            last_time = current_time

    else:
        HAL.setV(0)
        start_coordinates = None  # Reinicia las coordenadas iniciales si no hay contornos detectados

    GUI.showImage(red_mask)
    print('%d cX: %.2f cY: %.2f' % (i, cX, cY))
    i += 1
