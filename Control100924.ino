//LLamar a las librerias principales
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"
//Cinematica
int l1=9 ;
int l2=8;

int stepClearance=2;
int stepHeight=16;

int k01=10;
int k02=10;
int k03=10;

int Length=5;
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
int limitesA[14]= {0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int limitesB[14]= {90, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180};
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
  int AbsAngles[] = {
        validarAngulo(0, X0), validarAngulo(1, X1), validarAngulo(2, X2), validarAngulo(3, X3),
        validarAngulo(4, X4), validarAngulo(5, X5), validarAngulo(6, X6), validarAngulo(7, X7),
        validarAngulo(8, X8), validarAngulo(9, X9), validarAngulo(10, X10), validarAngulo(11, X11),
        validarAngulo(12, X12), validarAngulo(13, X13)
  };
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
    P0 = AbsAngles[0]; P1 = AbsAngles[1]; P2 = AbsAngles[2]; P3 = AbsAngles[3];
    P4 = AbsAngles[4]; P5 = AbsAngles[5]; P6 = AbsAngles[6]; P7 = AbsAngles[7];
    P8 = AbsAngles[8]; P9 = AbsAngles[9]; P10 = AbsAngles[10]; P11 = AbsAngles[11];
    P12 = AbsAngles[12]; P13 = AbsAngles[13];
/*for (int i = 0; i < CAM; i++) { //Diagnostic Mode
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
}
//Funcion de Movimiento Avanzado (Relativo)
void AdvMoveRel(int time, int steps,int X0,int X1,int X2,int X3,int X4,int X5,int X6,int X7,int X8,int X9,int X10,int X11,int X12,int X13) {
    int RelAngles[] = {
        validarAngulo(0, P0 + X0), validarAngulo(1, P1 + X1), validarAngulo(2, P2 + X2),
        validarAngulo(3, P3 + X3), validarAngulo(4, P4 + X4), validarAngulo(5, P5 + X5),
        validarAngulo(6, P6 + X6), validarAngulo(7, P7 + X7), validarAngulo(8, P8 + X8),
        validarAngulo(9, P9 + X9), validarAngulo(10, P10 + X10), validarAngulo(11, P11 + X11),
        validarAngulo(12, P12 + X12), validarAngulo(13, P13 + X13)
    };
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
    P0 = RelAngles[0]; P1 = RelAngles[1]; P2 = RelAngles[2]; P3 = RelAngles[3];
    P4 = RelAngles[4]; P5 = RelAngles[5]; P6 = RelAngles[6]; P7 = RelAngles[7];
    P8 = RelAngles[8]; P9 = RelAngles[9]; P10 = RelAngles[10]; P11 = RelAngles[11];
    P12 = RelAngles[12]; P13 = RelAngles[13];

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

 
//Editor de variables
int separatorIndex = command.indexOf(':');
  if (separatorIndex != -1) {
    String variableName = command.substring(0, separatorIndex);
    int newValue = command.substring(separatorIndex + 1).toInt();
    setVariable(variableName, newValue);
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
    takeStep(Length, t01);
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
  long Fleg;
  Fleg = l1 + l2;
  for (float i = Fleg; i >= stepHeight; i-=0.5){
    pos(0, i, 'l');
    pos(0, i, 'r');
  }
}
//Editor de variables
void setVariable(String variableName, int newValue) {
  // Actualizar variables individuales
  if (variableName == "T1") {
    t01 = newValue;
  } else if (variableName == "T2") {
    t02 = newValue;
  } else if (variableName == "T3") {
    t03 = newValue;
  } else if (variableName == "K1") {
    k01 = newValue;
  } else if (variableName == "K2") {
    k02 = newValue;
  } else if (variableName == "K3") {
    k03 = newValue;
  } else if (variableName == "l1") {
    l1 = newValue;
  } else if (variableName == "l2") {
    l2 = newValue;
  } else if (variableName == "stepClearance") {
    stepClearance = newValue;
  } else if (variableName == "stepHeight") {
    stepHeight = newValue;
  } else if (variableName == "Length") {
    Length = newValue;
  }else if (variableName.startsWith("P")) { // Variables P
    int index = variableName.substring(1).toInt();
    if (index >= 0 && index < 14) {
      int *pointers[] = { &P0, &P1, &P2, &P3, &P4, &P5, &P6, &P7, &P8, &P9, &P10, &P11, &P12, &P13 };
      *pointers[index] = newValue;
    }
  } else if (variableName.startsWith("posiciones[")) { // Array posiciones
    int startIdx = variableName.indexOf('[') + 1;
    int endIdx = variableName.indexOf(']');
    int index = variableName.substring(startIdx, endIdx).toInt();
    if (index >= 0 && index < 14) {
      posiciones[index] = newValue;
    }
  }
}
//verificador de angulos 
int validarAngulo(int servoID, int anguloSolicitado) {
    int minAngulo = min(limitesA[servoID], limitesB[servoID]);
    int maxAngulo = max(limitesA[servoID], limitesB[servoID]);

    // Si el ángulo está fuera del rango, ajustarlo al límite más cercano
    if (anguloSolicitado < minAngulo) {
        return minAngulo;
    } else if (anguloSolicitado > maxAngulo) {
        return maxAngulo;
    }
    return anguloSolicitado;
}
