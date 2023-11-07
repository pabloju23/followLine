from GUI import GUI
from HAL import HAL
import cv2
import numpy as np
import time
import math

# PID class
class PID:
    def __init__(self, kp, ki, kd, st):
        # Parámetros del controlador PID
        self.kp = kp  # Ganancia proporcional
        self.ki = ki  # Ganancia integral
        self.kd = kd  # Ganancia derivativa
        self.last_error = 0
        self.last_output = 0
        self.last_time = 0
        self.st = st
        self.last_sample = time.time()
        self.iterm = 0

    def set_lims(self, outmin, outmax):
        # Límites de la salida del controlador
        self.outmin = outmin
        self.outmax = outmax

    def calc(self, error):
        out = self.last_output
        # Calcula el tiempo que ha pasado
        diff_time = time.time() - self.last_sample

        if (diff_time >= self.st):
          # Parte derivativa
          diff_error = error - self.last_error

          # Parte integrativa (nunca más alta que max)
          self.iterm += error * self.ki
          if(self.iterm > self.outmax): self.iterm = self.outmax
          elif (self.iterm < self.outmin): self.iterm = self.outmin

          # Output (nunca más alto que max)
          out = self.kp * error + self.kd * diff_error + self.iterm
          if(out  > self.outmax): out = self.outmax
          elif(out < self.outmin): out = self.outmin

          # Almacena informaicon para la proxima vez
          self.last_error = error
          self.last_output = out
          self.last_sample = time.time()

        return out

class ErrBuff:
    def __init__(self, max_size):
        # Tamaño máximo del búfer de errores
        self.max_size = max_size

        # Inicializar el búfer de errores
        self.buffer = []

    def add_error(self, error):
        # Agregar un nuevo error al búfer
        self.buffer.append(error)

        # Si el búfer supera el tamaño máximo, eliminar el error más antiguo
        if len(self.buffer) > self.max_size:
            self.buffer.pop(0)

    def get_errors(self):
        # Obtener todos los errores almacenados en el búfer
        return self.buffer

    def calculate_average(self):
        # Calcular el promedio de los errores en el búfer
        if self.buffer:
            return sum(self.buffer) / len(self.buffer)
        else:
            return None

    def clear_buffer(self):
        # Borrar todos los errores del búfer
        self.buffer = []

# Definir los parámetros del control PID para la dirección
kp_steering = 0.01
ki_steering = 0.001
kd_steering = 0.001
max_steering_angle = 0.5  # Máximo ángulo de dirección permitido
normal_speed = 1
curve_speed = 0.5

# Crear una instancia de la clase PID para el control de dirección
steering_pid = PID(kp_steering, ki_steering, kd_steering, st=0.1)  # Ajusta st según tu necesidad

# En el bucle principal
while True:
    img = HAL.getImage()

    # Detectar la carretera y calcular el error de dirección (debes implementar esto)

    # Calcular el error de dirección basado en la detección de la carretera
    road_error = 0  # Debes calcular el error adecuadamente

    # Configurar los límites de salida para el controlador de dirección
    steering_pid.set_lims(-max_steering_angle, max_steering_angle)

    # Control de dirección (PID)
    steering_output = steering_pid.calc(road_error)

    # Control de velocidad en las curvas
    if abs(steering_output) > 0.1:  # Detectar una curva (ajusta el valor según sea necesario)
        HAL.setV(curve_speed)
    else:
        HAL.setV(normal_speed)

    # Control de dirección del coche
    HAL.setW(steering_output)


# Realizar análisis o cálculos adicionales basados en los errores almacenados
average_error = ErrBuff.calculate_average()
print("Promedio de errores:", average_error)


