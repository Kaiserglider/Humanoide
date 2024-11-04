//LLamar a las librerias principales
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"
//Cinematica
#define hipLOffset 91
#define kneeLOffset 40
#define ankleLOffset 135
#define hipROffset 90
#define kneeROffset 140
#define ankleROffset 45

#define l1 9
#define l2 8

#define stepClearance 2
#define stepHeight 16
int k01=10;
int k02=10;
int k03=10;
/*#include <SPI.h>
#include <SD.h>

//Definir pines y direccion de Giroscopio (MPU6050)
#define MPU6050_ADDR 0x68 //Cambiar si es necesario
#define SD_CS 5 //Pin para chip SD para poder escribir archivo CSV y obtener datos del giroscopio
*/
// Definir modulo PWM
#define PCA9685_ADDR 0x40  // Dirección I2C del PCA9685
//Definir Variables de posiciones Iniciales
int posiciones[14] = {45, 140, 90, 135, 40, 91, 20, 160, 160, 100, 90, 20, 80, 90};
//Definir Variables de posiciones variables
int P0 = 45;
int P1 = 140;
int P2 = 90;
int P3 = 135;
int P4 = 40;
int P5 = 91;
int P6 = 20;
int P7 = 160;
int P8 = 160;
int P9 = 100;
int P10 = 90;
int P11 = 20;
int P12 = 80;
int P13 = 90; 
//Definir variable para control AdvMove
int servos[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13};
int CAM=13;
//Definir tiempos
int t01 = 20;
int t02 = 10;
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
//cinematica
//  initialize();
/*
  //Iniciar Giroscopio
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); //Registo de potencia
  Wire.write(0); //Levantar MPU6050(Giroscopio)
  Wire.endTransmission(true);
  
  //Iniciar tarjeta SD
  if(!SD.begin(SD_CS)){
    Serial.println("Error al iniciar tarjeta SD");
    return;
  }
  */
}

void loop() {
/*
  //Leer Giroscopio
  int16_t ax,ay,az, gx,gy,gz;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); //Direccion del primer registro de datos
  Wire.endTransmission(false);
  Wire.requireForm(MPU6050_ADDR, 14, true); //14 registro de lectura
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  //Guardar datos en archivo CSV (Se usara una libreria en Python llamda Pandas para leer los datos y encontrar tendencia y talvez implementar una IA para optimizar los angulos)
  saveDataToCSV(ax,ay,ax,gx,gy,gz);
*/
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
   AdvMoveAbs(100,10,posiciones[0],posiciones[1],posiciones[2],posiciones[3],posiciones[4],posiciones[5],posiciones[6],posiciones[7],posiciones[8],posiciones[9],posiciones[10],posiciones[11],posiciones[12],posiciones[13]);
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
//Funcion de Movimiento avanzado (Absoluto)
void AdvMoveAbs(int time, int steps,int X0,int X1,int X2,int X3,int X4,int X5,int X6,int X7,int X8,int X9,int X10,int X11,int X12,int X13) {
  int AbsAngles[]= {X0,X1,X2,X3,X4,X5,X6,X7,X8,X9,X10,X11,X12,X13};
  int startAngles[]={P0,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13};
/*  Serial.print(X0); //Diagnostic Mode
  Serial.println();
  for (int i = 0; i < CAM; i++) {
    Serial.print(startAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
  smoothMove(14, servos, startAngles, AbsAngles, time);
 // Array de Pulsos Iniciales y Finales
    P0=X0;
    P1=X1;
    P2=X2;
    P3=X3;
    P4=X4;
    P5=X5;
    P6=X6;
    P7=X7;
    P8=X8;
    P9=X9;
    P10=X10;
    P11=X11;
    P12=X12;
    P13=X13;
/*for (int i = 0; i < CAM; i++) { //Diagnostic Mode
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
}
//Funcion de Movimiento Avanzado (Relativo)
void AdvMoveRel(int time, int steps,int X0,int X1,int X2,int X3,int X4,int X5,int X6,int X7,int X8,int X9,int X10,int X11,int X12,int X13) {
  int RelAngles[]= {X0,X1,X2,X3,X4,X5,X6,X7,X8,X9,X10,X11,X12,X13};
  int startAngles[]={P0,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13};
  //Serial.print(X0); //Diagnostic Mode
  //Serial.println();
  for (int i = 0; i < CAM; i++) {
    //Serial.print(startAngles[i]);
    //Serial.write(Str);
    RelAngles[i] =startAngles[i] + RelAngles[i];
  }
  //Serial.println();
  smoothMove(14, servos, startAngles, RelAngles, time);
 /*for (int i = 0; i < CAM; i++) { //Diagnostic Mode
    Serial.print(RelAngles[i]);
    Serial.write(Str);
  }
  Serial.println();*/
    P0=X0+P0;
    P1=X1+P1;
    P2=X2+P2;
    P3=X3+P3;
    P4=X4+P4;
    P5=X5+P5;
    P6=X6+P6;
    P7=X7+P7;
    P8=X8+P8;
    P9=X9+P9;
    P10=X10+P10;
    P11=X11+P11;
    P12=X12+P12;
    P13=X13+P13;

}
/*
void saveDataToCSV(int16_t ax, int16_t ay, int16_t ax, int16_t gx, int16_t gy, int16_t gz){
  File dataFile = SD.open("datos.csv", FILE_APPEND);
  if (dataFile){
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.print(gx);
    dataFile.print(",");
    dataFile.print(gy);
    dataFile.print(",");
    dataFile.print(gz);
    dataFile.close();
  }
  else{
    Serial.println("Error al abrir el archivo")
  }
}
*/
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
if (command.startsWith("K1:")) {
    int newValue = command.substring(3).toInt(); // Extraer el valor 
    if (newValue > 0) {
      k01 = newValue; // Actualizar el valor
    } else {
    }
    return;
  }
 if (command.startsWith("K2:")) {
    int newValue = command.substring(3).toInt(); // Extraer el valor
    if (newValue > 0) {
      k02 = newValue; // Actualizar el valor
    } else {
    }
    return;
  }
   if (command.startsWith("K3:")) {
    int newValue = command.substring(3).toInt(); // Extraer el valor
    if (newValue > 0) {
      k03 = newValue; // Actualizar el valor
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
    derecha2();
    delay(1000);
    derecha2();
    delay(1000);
    delay(10);
}
if (command.startsWith("cinematic")) {
  //cinematica
    takeStep(5, t01);
}
if (command.startsWith("prepare")) {
  //cinematica
    initialize();
}
if (command.startsWith("derecha")) {
   derecha2();
}
}
/* 
   delay (5000);
   int servos02[] = { 0, 1, 2, 3, 4, 5, 6, 7,8,9, 10, 11, 12, 13};
   int startAngles02[] = { 25, 30, 180, 165, 160, 5, 1, 179, 135, 70, 90, 45, 110, 90 };
   int endAngles02[] = {V0, V1, V2, V3, V4, V5 ,V6, V7, V8, V9, V10, V11, V12, V13};
   smoothMove(14, servos02, startAngles02, endAngles02, 30);
*/



//Secuencias de Movimiento

void derecha2 () {
AdvMoveRel(20,10,0,-30,30,5,0,0,0,-5,0,0,0,0,0,0);
delay(100);
AdvMoveRel(20,10,7,0,20,0,0,5,70,0,0,0,0,0,0,0);
delay(100);
AdvMoveRel(20,10,15,20,0,0,0,0,-20,0,0,0,0,0,0,0);
delay(100);
}

//cinematica
void updateServoPos(int target1, int target2, int target3, char leg){
  if (leg == 'l'){
    AdvMoveAbs(t01,10,P0,P1,P2,posiciones[3]-(target3-90-k01),posiciones[4]+ target2+k02,posiciones[5]+target1+k03,P6,P7,P8,P9,P10,P11,P12,P13);

  }
  else if (leg == 'r'){ 
    AdvMoveAbs(t01,10, posiciones[0]+(target3-90-k01), posiciones[1]-target2-k02,posiciones[2]-target1-k03,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13);
    
  }
}

void pos(float x, float z, char leg){
  float hipRad2 = atan(x/z);
  float hipDeg2 = hipRad2 * (180/PI);

  float z2 = z/cos(hipRad2);

  float hipRad1 = acos((sq(l1) + sq(z2) - sq(l2))/(2*l1*z2));
  float hipDeg1 = hipRad1 * (180/PI);
  
  float kneeRad = PI - acos((sq(l1) + sq(l2) - sq(z2))/(2*l1*l2));

  float ankleRad = PI/2 + hipRad2 - acos((sq(l2) + sq(z2) - sq(l1))/(2*l2*z2));
  
  float hipDeg = hipDeg1 + hipDeg2;
  float kneeDeg = kneeRad * (180/PI);
  float ankleDeg = ankleRad * (180/PI);
/*
  Serial.print(leg);
  Serial.print("\t");
  Serial.print(ankleDeg);
  Serial.print("\t");
  Serial.print(kneeDeg);
  Serial.print("\t");
  Serial.print(hipDeg);
  Serial.print("\t");
*/
  updateServoPos(hipDeg, kneeDeg, ankleDeg, leg);  
}

void takeStep(float stepLength, int stepVelocity){
  for (float i = stepLength; i >= -stepLength; i-=0.5){
    pos(i, stepHeight, 'r');
    pos(-i, stepHeight - stepClearance, 'l');
    delay(stepVelocity);
  }

  for (float i = stepLength; i >= -stepLength; i-=0.5){
    pos(-i, stepHeight - stepClearance, 'r');
    pos(i, stepHeight, 'l');
    delay(stepVelocity);
  }
}
void initialize(){
  for (float i = 17; i >= stepHeight; i-=0.5){
    pos(0, i, 'l');
    pos(0, i, 'r');
  }
}
