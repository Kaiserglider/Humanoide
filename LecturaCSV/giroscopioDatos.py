import pandas as pd
import numpy as np
import random
#Leer archivo CSV
data = pd.read_csv("datos.csv", header=None, names=["ax", "ay", "az", "gx", "gy", "gz"])

#Calcular las medias de los giros
mean_gx = np.mean(data["gx"])
mean_gy = np.mean(data["gy"])
mean_gz = np.mean(data["gz"])

print("Media de giroscopio en X: ", mean_gx)
print("Media de giroscopio en Y: ", mean_gy)
print("Media de giroscopio en Z: ", mean_gz)