
import numpy as np
import random

#Algoritmo basico de refuerzo
#Parametros
angle_min = -90 #Angulo minimo
angle_max = 90 #Angulo maximo
angle_step = 10 #pasos de ajuste de angulos
num_angles = (angle_max- angle_max) // angle_step +1 #numero de angulos
num_actions = 3 #ajustes
learning_rate = 0.1
discount_factor = 0.95
epsilon = 1.0
epsilon_decay = 0.99
min_epsilon = 0.1

#Iniciar Q-Learning
q_table = np.zeros((num_angles, num_actions))

#Funcion para elegir accion usando politica epsilon-greedy
def choose_action(state):
    if random.uniform(0,1) < epsilon:
        return random.randint(0, num_actions -1) #Exploracion
    else:
        return np.argmax(q_table[state]) #Explotacion
    
#Funcion para obtener recompenza
def get_reward(angle, stable_angle):
    #Definir angulo que se considera estable
    if angle == stable_angle:
        return 10 #Recompenza positiva
    else:
        return -1 #Penalizacion si el angulo no es estable

#Entrenamiento
for episode in range(1000): #Numero de episodios de entrenamiento
    #Estado inicial aleatorio
    inizial_angle = random.randint(angle_min, angle_max) // angle_step
    state = inizial_angle #Estado inicial en base a la discretizacion
    done = False

    while not done:
        action = choose_action(state) #Eligir accion
        #Calcular nuevo estado basado en accion
        new_angle = angle_min +(state * angle_step) +(action -1) *angle_step
        new_angle = max(angle_min, min(new_angle, angle_max)) #Limita a los angulos permitidos
        new_state = (new_angle - angle_min) // angle_step #Estado siquiente

        #Medir estabilidad del angulo
        stable_angle = 0 #Define el angulo deseado como estable
        reward = get_reward(new_angle, stable_angle) #Obtener recompenza

        #Actualizar Q-valor
        best_future_q = np.max(q_table[new_state])
        q_table[state, action] += learning_rate *(reward + discount_factor + best_future_q - q_table[state, action])

        #Cambiar a nuevo estado
        state = new_state

        #Finializar el episodio si se alcanza un estado terminal
        if state == (angle_max - angle_min) // angle_step:
            done = True
    
    #Actualizar epsilon
    if epsilon > min_epsilon:
        epsilon *= epsilon_decay

#Imprimir tabla Q Final
print("Tabla Q Final:")
print(q_table)
