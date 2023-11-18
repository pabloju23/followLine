from GUI import GUI
from HAL import HAL
import cv2
import time

# Parámetros para rectas
straight_params = {
    'kp': 0.003,
    'ki': 0.000001,
    'kd': 0.001,
    'straight_speed_factor': 0,
    'min_speed': 2,
    'initial_speed': 5
}

# Parámetros para curvas
curve_params = {
    'kp': 0.005,
    'ki': 0.000001,  
    'kd': 0.005,
    'curve_speed_factor': 0.2,
    'min_speed': 1.5,
    'initial_speed': 3
}

# Inicialización de variables del PID
last_error = 0
integral = 0
anti_windup_limit = 5
last_time = time.time()

# Guarda las coordenadas iniciales
start_time = time.time()
start_coordinates = None
completion_time_threshold = 10  # Tiempo mínimo para considerar que el coche ha completado el circuito
x_margin = 10  # Margen en la coordenada x
y_margin= 0
threshold_for_straight = 10
in_curve=False
i = 0


def calculate_speed_factor(curve_angle):
    return max(0.5, 1 - 0.001 * abs(curve_angle))


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
            err = (320 - cX)

            # Calcula el tiempo que ha pasado
            current_time = time.time()
            dt = current_time - last_time
            
            # Calcula el ángulo de la curva
            ellipse = cv2.fitEllipse(max_contour)
            curve_angle = ellipse[2]

            # Parametros para rectas y curvas
            if abs(curve_angle) < threshold_for_straight:
                params = straight_params
                in_curve=False
            else:
                params = curve_params
                in_curve=True
                
            # Parte proporcional
            proportional = params['kp'] * err
            integral += params['ki'] * err * dt
            # Parte integral con Anti-Windup
            if integral >= 0:
               # integral += params['ki'] * err * dt
                print('integral: ', integral)
                integral = max(min(integral, anti_windup_limit), -anti_windup_limit)
            else:
                integral = 0  # Reiniciar la integral si hay cambio de signo en el error


            # Parte derivativa
            derivative = params['kd'] * (err - last_error) / dt

            # Calcula la salida del PID
            output = proportional + integral + derivative
            
            # Ajusta la velocidad en función del ángulo de la curva
            speed_factor = calculate_speed_factor(curve_angle)
            
            if in_curve:
                # Si está en una curva, ajusta la velocidad considerando el límite inferior y la velocidad inicial
                speed = max(params['min_speed'], params['initial_speed'] * speed_factor * (1 - params['curve_speed_factor'] * abs(output)))
            else:
                # Si está en una recta, ajusta la velocidad considerando el límite inferior y la velocidad inicial
                speed = max(params['min_speed'], params['initial_speed'] * speed_factor * (1 - params['straight_speed_factor'] * abs(output)))


            # Ajustar la velocidad y el ángulo de dirección en función de la salida del PID y la velocidad
            HAL.setV(speed)
            print('v_lineal: ', speed)
            HAL.setW(0.6 * output * speed_factor)

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
