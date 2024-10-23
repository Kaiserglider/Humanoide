//LLamar a las librerias principales
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"

// Definir modulo PWM
#define PCA9685_ADDR 0x40  // Dirección I2C del PCA9685

//Definir Variables de posiciones Iniciales
int posiciones[14] = {28, 55, 93, 155, 130, 95, 20, 160, 160, 100, 90, 20, 80, 90};

//Definir tiempos
int t01 = 20;
int t02 = 30;
int t03 = 35;     // Tiempo de retraso entre movimientos
int steps = 10;  // Número de pasos para suavizar el movimiento
//Variables para control manual de motores
int N = 100;
int M = 100;
int P = 0;

//valores min y maximos del pulso
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
uint16_t servoMin = 500;   // Pulso "mínimo" para el servomotor
uint16_t servoMax = 3400;  // Pulso "máximo" para el servomotor

BluetoothSerial SerialBT;
String device_name = "Dorado";

void setup() {
  Serial.begin(115200);
  SerialBT.begin(device_name);  // Nombre del dispositivo Bluetooth
  Serial.printf("El dispositivo con nombre \"%s\" está iniciado.\n¡Ahora puedes emparejarlo con Bluetooth!\n", device_name.c_str());

  while (!Serial) {
    // Espera a que el puerto serie esté listo
  }

  pwm.begin();
  pwm.setPWMFreq(330);  // Configura la frecuencia PWM a 330 Hz para servomotores
  setInitialServoPositions();
}

void loop() {
  // Verificar si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Verificar si hay datos disponibles en el puerto Bluetooth
  if (SerialBT.available() > 0) {
    String command = SerialBT.readStringUntil('\n');
    processCommand(command);
  }
}
//Convertir angulo a ancho de pulso del servo
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, servoMin, servoMax);
  return pulse;
}

void setInitialServoPositions() {
    for (int i = 0; i < 14; i++) {
        pwm.setPWM(i, 0, angleToPulse(posiciones[i])); //Utiliza el array posiciones
    }
}

//Funcion de movimiento
void smoothMove(int count, int servos[], int startAngles[], int endAngles[], int time) {
  //Array de Pulsos Iniciales y Finales
  int pulsesStart[count];
  int pulsesEnd[count];
  int pulseSteps[count];

  //Calcula el pulso inicial, final y paso para cada servo
  for (int i = 0; i < count; i++) {
    pulsesStart[i] = angleToPulse(startAngles[i]);
    pulsesEnd[i] = angleToPulse(endAngles[i]);
    pulseSteps[i] = (pulsesEnd[i] - pulsesStart[i]) / steps;
  }

  //Mueve los servos en pasos
  for (int i = 0; i <= steps; i++) {
    for (int j = 0; j < count; j++) {
      int currentPulse = pulsesStart[j] + (pulseSteps[j] * i);
      pwm.setPWM(servos[j], 0, currentPulse);
    }
    delay(time / steps); //Divide el tiempo por los pasos para suavizar
  }
}

void processCommand(String command) {
  command.trim();  // Elimina espacios en blanco al inicio y al final
  //Modificador de Velocidades
  if (command.startsWith("T1:")) {
    int newValue = command.substring(3).toInt(); // Extraer el valor después de "T1="
    if (newValue > 0) {
      t01 = newValue; // Actualizar el valor de t01
    } else {
    }
    return;
  }
 if (command.startsWith("T2:")) {
    int newValue = command.substring(3).toInt(); // Extraer el valor después de "T1="
    if (newValue > 0) {
      t02 = newValue; // Actualizar el valor de t01
    } else {
    }
    return;
  }
   if (command.startsWith("T3:")) {
    int newValue = command.substring(3).toInt(); // Extraer el valor después de "T1="
    if (newValue > 0) {
      t03 = newValue; // Actualizar el valor de t01
    } else {
    }
    return;
  }

// modo manual para mover los servos
  if (command.startsWith("M")) {
    int nV = command.substring(1).toInt(); // Extrae
    int nP = command.substring(3).toInt(); 
    if (nV >= 0) {
        M = nV; // Actualizar el valor 
        P = nP;
    } else {
    }
    pwm.setPWM(M, 0, angleToPulse(P));
return;
}
  // Dividir el comando por ';'
  int startIndex = 0;
  int endIndex = command.indexOf(';');

  while (endIndex >= 0) {
    processSingleCommand(command.substring(startIndex, endIndex));
    startIndex = endIndex + 1;
    endIndex = command.indexOf(';', startIndex);
  }

  // Procesar el último comando si existe
  if (startIndex < command.length()) {
    processSingleCommand(command.substring(startIndex));
  }
}

void processSingleCommand(String command) {
  
if (command.startsWith("firmes")) {
setInitialServoPositions();
}

if (command.startsWith("caminar")) {
    derecha();
    delay(1000);
    derecha();
    delay(1000);
    izquierda();
    delay(10);
}
if (command.startsWith("izquierdo")) {
    izquierda();
}
if (command.startsWith("derecha")) {
    derecha();
}
if (command.startsWith("girar")) {
    girar();
}
if (command.startsWith("sentar")) {
    sentar();
/* 
   delay (5000);
   int servos02[] = { 0, 1, 2, 3, 4, 5, 6, 7,8,9, 10, 11, 12, 13};
   int startAngles02[] = { 25, 30, 180, 165, 160, 5, 1, 179, 135, 70, 90, 45, 110, 90 };
   int endAngles02[] = {V0, V1, V2, V3, V4, V5 ,V6, V7, V8, V9, V10, V11, V12, V13};
   smoothMove(14, servos02, startAngles02, endAngles02, 30);
*/
}

}
//Secuencias de Movimiento
void sentar(){
  delay(1000);
    int servos00[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
    int startAngles00[] = {posiciones[0], posiciones[1], posiciones[2], posiciones[3], posiciones[4], posiciones[5], posiciones[6], posiciones[7], posiciones[8], posiciones[9], posiciones[10], posiciones[11], posiciones[12], posiciones[13]};
    int endAngles00[] = { 25, 30, 180, 165, 160, 5, 1, 179, 135, 70, 90, 45, 110, 90 };
    smoothMove(14, servos00, startAngles00, endAngles00, t01);
}
void izquierda(){
    int servos06[] = { 4, 5, 0, 6};
    int startAngles06[] = {130, 95, 28, 20};
    int endAngles06[] = { 160, 65, 20, 45};
    smoothMove(4, servos06, startAngles06, endAngles06, t01);

    int servos07[] = { 5, 2, 3 };
    int startAngles07[] = {65, 93, 155};
    int endAngles07[] = { 45, 98, 145};
    smoothMove(3, servos07, startAngles07, endAngles07, t01);

    int servos08[] = { 4, 3};
    int startAngles08[] = {155, 145};
    int endAngles08[] = { 135, 130};
    smoothMove(2, servos08, startAngles08, endAngles08, t01);

    int servos09[] = { 5, 3, 0 };
    int startAngles09[] = {45, 130, 20};
    int endAngles09[] = { 55, 140, 15 };
    smoothMove(3, servos09, startAngles09, endAngles09, t03);

    int servos10[] = { 5, 3, 4, 2, 0, 6};
    int startAngles10[] = {55, 140, 135, 98, 15, 45};
    int endAngles10[] = { 85, 155, 130, 93, 35, 0};
    smoothMove(6, servos10, startAngles10, endAngles10,t03);
    delay(20);
    pwm.setPWM(0, 0, angleToPulse(posiciones[1]));
    delay(200);

}

void derecha () {
    int servos01[] = { 1, 2, 3, 7 };
    int startAngles01[] = {55, 93, 155, 160};
    int endAngles01[] = { 25, 123, 160, 155 };
    smoothMove(4, servos01, startAngles01, endAngles01, t01);

    int servos02[] = { 2, 5, 0, 6};
    int startAngles02[] = {123, 95, 28, 20};
    int endAngles02[] = { 143, 90, 35, 90};
    smoothMove(4, servos02, startAngles02, endAngles02, t01);

    int servos03[] = { 1, 0, 6};
    int startAngles03[] = {25, 35, 90};
    int endAngles03[] = { 45, 50, 70};
    smoothMove(3, servos03, startAngles03, endAngles03, t01);

    int servos04[] = { 2, 0, 3, 6};
    int startAngles04[] = {143, 50, 160, 70};
    int endAngles04[] = { 133, 50, 165, 50};
    smoothMove(4, servos04, startAngles04, endAngles04, t02);

    int servos05[] =      {2, 0, 1, 5, 3, 7, 6};
    int startAngles05[] = {123, 50, 45, 90, 165, 155, 50};
    int endAngles05[] =   {93, 28, 50, 95, 155, 160, 20};
    smoothMove(7, servos05, startAngles05, endAngles05, t02);
}

void girar () {
    int servos01[] = { 1, 2, 3, 7 };
    int startAngles01[] = {55, 93, 155, 160};
    int endAngles01[] = { 25, 123, 160, 155 };
    smoothMove(4, servos01, startAngles01, endAngles01, t01);

    int servos02[] = { 2, 5, 0, 6};
    int startAngles02[] = {123, 95, 28, 20};
    int endAngles02[] = { 143, 90, 35, 90};
    smoothMove(4, servos02, startAngles02, endAngles02, t01);

    int servos03[] = { 1, 0, 6};
    int startAngles03[] = {25, 35, 90};
    int endAngles03[] = { 45, 50, 70};
    smoothMove(3, servos03, startAngles03, endAngles03, t01);

    int servos04[] = { 2, 0, 3, 6};
    int startAngles04[] = {143, 50, 160, 70};
    int endAngles04[] = { 133, 50, 165, 50};
    smoothMove(4, servos04, startAngles04, endAngles04, 15);

    int servos05[] =      {2, 0, 1, 5, 3, 7, 6};
    int startAngles05[] = {123, 50, 45, 90, 165, 155, 50};
    int endAngles05[] =   {93, 28, 50, 95, 155, 160, 20};
    smoothMove(7, servos05, startAngles05, endAngles05, 15);
}
