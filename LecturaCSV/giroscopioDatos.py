import pandas as pd
import numpy as np
import random
#Leer archivo CSV
data = pd.read_csv("datos.csv", header=None, names=["ax", "ay", "az", "gx", "gy", "gz", "angle0", "angle1", "angle2","angle3","angle4","angle5","angle6","angle7","angle8","angle9","angle10","angle11","angle12","angle13"])

#Calcular las medias de los giros
mean_gx = np.mean(data["gx"])
mean_gy = np.mean(data["gy"])
mean_gz = np.mean(data["gz"])

print("Media de giroscopio en X: ", mean_gx)
print("Media de giroscopio en Y: ", mean_gy)
print("Media de giroscopio en Z: ", mean_gz)

#Calcular las medias de los angulos:
mean_servo0 = np.mean(data["angle0"])
mean_servo1 = np.mean(data["angle1"])
mean_servo2 = np.mean(data["angle2"])
mean_servo3 = np.mean(data["angle3"])
mean_servo4 = np.mean(data["angle4"])
mean_servo5 = np.mean(data["angle5"])
mean_servo6 = np.mean(data["angle6"])
mean_servo7 = np.mean(data["angle7"])
mean_servo8 = np.mean(data["angle8"])
mean_servo9 = np.mean(data["angle9"])
mean_servo10 = np.mean(data["angle10"])
mean_servo11 = np.mean(data["angle11"])
mean_servo12 = np.mean(data["angle12"])
mean_servo13 = np.mean(data["angle13"])

print("Media del ángulo del Servo 0: ", mean_servo0)
print("Media del ángulo del Servo 1: ", mean_servo1)
print("Media del ángulo del Servo 2: ", mean_servo2)
print("Media del ángulo del Servo 3: ", mean_servo3)
print("Media del ángulo del Servo 4: ", mean_servo4)
print("Media del ángulo del Servo 5: ", mean_servo5)
print("Media del ángulo del Servo 6: ", mean_servo6)
print("Media del ángulo del Servo 7: ", mean_servo7)
print("Media del ángulo del Servo 8: ", mean_servo8)
print("Media del ángulo del Servo 9: ", mean_servo9)
print("Media del ángulo del Servo 10: ", mean_servo10)
print("Media del ángulo del Servo 11: ", mean_servo11)
print("Media del ángulo del Servo 12: ", mean_servo12)
print("Media del ángulo del Servo 13: ", mean_servo13)
