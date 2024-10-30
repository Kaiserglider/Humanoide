import serial
import csv
import time

#Configura el puerto Bluetooth en la computadora (ajusta el nombre del puerto)
bt_port = ""  # Ajusta al puerto correcto de tu dispositivo
baud_rate = 115200

#Abre la conexion
ser = serial.Serial(bt_port, baud_rate, timeout=1)

#Crea el archivo CSV
with open("datos_mpu6050_servos.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["ax", "ay", "az", "gx", "gy", "gz", "servo1_angle", "servo2_angle"])

    try:
        while True:
            #Lee datos del ESP32
            line = ser.readline().decode().strip()
            if line:
                print("Datos recibidos:", line)
                data = line.split(",")
                #Escribe en el archivo CSV
                writer.writerow(data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Finalizado por el usuario.")

#Cierra la conexion
ser.close()
