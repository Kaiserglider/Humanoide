//llamar a la libreria del bluetooth
#include "BluetoothSerial.h"
//LLamar a las librerias del modulo PWM
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//llamar a las librerias de los giroscopios
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//llamarlibrerias al PID y definirlo
#include <PIDController.hpp>
PID::PIDParameters<double> parameters(4.0, 0.2, 1);
PID::PIDController<double> pidController(parameters);
int XG1P =100 ;
int XG1O =0 ;
// Definir modulo PWM
#define PCA9685_ADDR 0x40  // Dirección I2C del PCA9685
//definimos giroscopio y variables
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
int XG1 = 0;
int YG1 = 0;
int ZG1 = 0;

int XG2 = 0;
int YG2 = 0;
int ZG2 = 0;
//valores min y maximos del pulso
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
uint16_t servoMin = 500;   // Pulso "mínimo" para el servomotor
uint16_t servoMax = 3400;  // Pulso "máximo" para el servomotor
//definicion del bluetooth
BluetoothSerial SerialBT;
String device_name = "Dorado";

TaskHandle_t Task1;
TaskHandle_t Task2;
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
int servos2[]={0,1,2,3,4,5};
int CAM2=6;
int servos3[]={6,7,8,9,10,11,12,13};
int CAM3=9;
//Definir tiempos
int t01 = 20;
int t02 = 10;
int t03 = 35;     // Tiempo de retraso entre movimientos
int steps = 10;  // Número de pasos para suavizar el movimiento
//Variables para control manual de motores
int N = 100;
int M = 100;
int P = 0;
//Cinematica
int l1=9 ;
int l2=8;

int stepClearance=2;
int stepHeight=16;

int k01=10;
int k02=10;
int k03=10;

int Length=5;

void setup() {
  Serial.begin(115200); 
  SerialBT.begin(device_name);  // Nombre del dispositivo Bluetooth
  Serial.printf("El dispositivo con nombre \"%s\" está iniciado.\n¡Ahora puedes emparejarlo con Bluetooth!\n", device_name.c_str());

  while (!Serial) {
       delay(10);// Espera a que el puerto serie esté listo
  }
  pwm.begin();
  pwm.setPWMFreq(330);
  setInitialServoPositions();
// inicialisamos giroscopio
if (!mpu1.begin(0x68)) {
  Serial.println("Sensor 1 init failed");
  while (1)
    yield();
}
Serial.println("MPU6050 1 Found!");
if (!mpu2.begin(0x69)) {
  Serial.println("Sensor 2 init failed");
  while (1)
    yield();
}
Serial.println("MPU6050 2 Found!");

mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
Serial.print("Gyro range set to: ");
switch (mpu1.getGyroRange()) {
case MPU6050_RANGE_250_DEG:
  Serial.println("+- 250 deg/s");
  break;
case MPU6050_RANGE_500_DEG:
  Serial.println("+- 500 deg/s");
  break;
case MPU6050_RANGE_1000_DEG:
  Serial.println("+- 1000 deg/s");
  break;
case MPU6050_RANGE_2000_DEG:
  Serial.println("+- 2000 deg/s");
  break;
}
mpu2.setFilterBandwidth(MPU6050_BAND_5_HZ);
mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
Serial.print("Gyro range set to: ");
switch (mpu2.getGyroRange()) {
case MPU6050_RANGE_250_DEG:
  Serial.println("+- 250 deg/s");
  break;
case MPU6050_RANGE_500_DEG:
  Serial.println("+- 500 deg/s");
  break;
case MPU6050_RANGE_1000_DEG:
  Serial.println("+- 1000 deg/s");
  break;
case MPU6050_RANGE_2000_DEG:
  Serial.println("+- 2000 deg/s");
  break;
}
mpu2.setFilterBandwidth(MPU6050_BAND_5_HZ);
  pidController.Input = XG1P;
  pidController.Setpoint = 100;

  pidController.TurnOn();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

//Task1code: check the MPU6050 And control arms
void Task1code( void * pvParameters ){
  Serial.print("Task1 started on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
 //   Serial.print("Task1 running on core ");
 //   Serial.println(xPortGetCoreID());
    MPU1();
    MPU2();
    Serial.print("G1 ");
    Serial.print(XG1);
    Serial.print(" ");
    Serial.print(YG1);
    Serial.print(" ");
    Serial.print(ZG1);
    Serial.print(" G2 ");
    Serial.print(XG2);
    Serial.print(" ");
    Serial.print(YG2);
    Serial.print(" ");
    Serial.println(ZG2);
    XG1P=100-XG1;
    pidController.Input = XG1P;
    pidController.Update();
    XG1O=pidController.Output;
     Serial.print(XG1P);
     Serial.print(" ");
     Serial.println(XG1O);
    
/*
    AdvMoveAbsA(20,10,P6+YG1,P7-YG1,P8+XG1,P9,P10,P11-XG1,P12,P13);
    delay(2500);*/
  } 
}

//Task2code: check the bluetooth and calculate cinemaatic
void Task2code( void * pvParameters ){
  Serial.print("Task2 started on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
 //   Serial.print("Task2 running on core ");
 //   Serial.println(xPortGetCoreID());
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Verificar si hay datos disponibles en el puerto Bluetooth
  if (SerialBT.available() > 0) {
    String command = SerialBT.readStringUntil('\n');
    processCommand(command);
  }
    /*
    Serial.print(" ");
    Serial.print("H");
    Serial.print(" ");
    delay(700);
    Serial.println("L");
    delay(700);*/
  }
}

void loop() {
  
}

//Convertir angulo a ancho de pulso del servo
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, servoMin, servoMax);
  return pulse;
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
      Serial.print("M:");
      Serial.print(servos[j]);
      Serial.write(" ");
      Serial.print("PWM:");
      Serial.print(currentPulse);
      Serial.write(" ");
    }
    Serial.println();
 //   Serial.print("Task2 running on core ");
 //   Serial.println(xPortGetCoreID());
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
void AdvMoveAbsL(int time, int steps,int X0,int X1,int X2,int X3,int X4,int X5) {
  
  int AbsAngles[] = {
        validarAngulo(0, X0), validarAngulo(1, X1), validarAngulo(2, X2), validarAngulo(3, X3),
        validarAngulo(4, X4), validarAngulo(5, X5)
  };
  int startAngles[]={P0,P1,P2,P3,P4,P5};

/*  Serial.print(X0); //Diagnostic Mode
  Serial.println();
  for (int i = 0; i < CAM; i++) {
    Serial.print(startAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
  smoothMove(CAM2, servos2, startAngles, AbsAngles, time);
 // Array de Pulsos Iniciales y Finales
    P0 = AbsAngles[0]; P1 = AbsAngles[1]; P2 = AbsAngles[2]; P3 = AbsAngles[3];
    P4 = AbsAngles[4]; P5 = AbsAngles[5];
/*for (int i = 0; i < CAM; i++) { //Diagnostic Mode
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
}
void AdvMoveAbsA(int time, int steps,int X6,int X7,int X8,int X9,int X10,int X11,int X12,int X13) {
  int AbsAngles[] = {
        validarAngulo(6, X6), validarAngulo(7, X7),
        validarAngulo(8, X8), validarAngulo(9, X9), validarAngulo(10, X10), validarAngulo(11, X11),
        validarAngulo(12, X12), validarAngulo(13, X13)
  };
  int startAngles[]={P6,P7,P8,P9,P10,P11,P12,P13};

/*  Serial.print(X0); //Diagnostic Mode
  Serial.println();
  for (int i = 0; i < CAM; i++) {
    Serial.print(startAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
  smoothMove(CAM3, servos3, startAngles, AbsAngles, time);
 // Array de Pulsos Iniciales y Finales
    P6 = AbsAngles[6]; P7 = AbsAngles[7];
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
    int separatorIndex = command.indexOf(","); // Busca la posición de la coma
    if (separatorIndex != -1) { // Si se encuentra una coma en el comando
        int nV = command.substring(1, separatorIndex).toInt(); // Extrae el valor del motor
        int nP = command.substring(separatorIndex + 1).toInt(); // Extrae el ángulo

        if (nV >= 0) { // Verifica que el número de motor sea válido
            M = nV; // Actualiza el número de motor
            P = nP; // Actualiza el ángulo

            pwm.setPWM(M, 0, angleToPulse(P)); // Envía el comando PWM

            Serial.print("Motor number: ");
            Serial.println(M);
            Serial.print("to: ");
            Serial.println(P);
        } else {
            Serial.println("Invalid motor number.");
        }
    } else {
        Serial.println("Invalid command format.");
    }
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
if (command.startsWith("in1leg")) {
 leg_1();
}
if (command.startsWith("cinematic")) {
  //cinematica
    takeStep(Length, t01);
    Serial.println("caminando");
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
void setInitialServoPositions() {
   AdvMoveAbs(100,10,posiciones[0],posiciones[1],posiciones[2],posiciones[3],posiciones[4],posiciones[5],posiciones[6],posiciones[7],posiciones[8],posiciones[9],posiciones[10],posiciones[11],posiciones[12],posiciones[13]);
}

void derecha2 () {
AdvMoveRel(20,10,0,-30,30,5,0,0,0,-5,0,0,0,0,0,0);
delay(100);
AdvMoveRel(20,10,7,0,20,0,0,5,70,0,0,0,0,0,0,0);
delay(100);
AdvMoveRel(20,10,15,20,0,0,0,0,-20,0,0,0,0,0,0,0);
delay(100);
}
void leg_1 () {
AdvMoveAbs(20,10,45,90,180,150,55,91,20,160,160,100,90,20,80,90);
}

//cinematica
void updateServoPos(int target1, int target2, int target3, char leg){
  if (leg == 'l'){
    AdvMoveAbsL(t01,10,P0,P1,P2,posiciones[3]-(target3-90-k01),posiciones[4]+ target2+k02,posiciones[5]+target1+k03);

  }
  else if (leg == 'r'){ 
    AdvMoveAbsL(t01,10, posiciones[0]+(target3-90-k01), posiciones[1]-target2-k02,posiciones[2]-target1-k03,P3,P4,P5);
    
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
    Serial.print("T1  updated:");
    Serial.println(t01);
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
    Serial.print("L2  updated:");
    Serial.println(l2);
  } else if (variableName == "stepClearance") {
    stepClearance = newValue;
  } else if (variableName == "stepHeight") {
    stepHeight = newValue;
  } else if (variableName == "Length") {
    Length = newValue;
  } else if (variableName == "steps") {
    steps = newValue;
  } else if (variableName.startsWith("P")) { // Variables P
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
void MPU1(){
    sensors_event_t a, g, temp;
    mpu1.getEvent(&a, &g, &temp);
    XG1 = g.gyro.x;
    YG1 = g.gyro.y;
    ZG1 = g.gyro.z;
    //diagnostic Mode
//    Serial.print("Task1 running on core ");
//    Serial.println(xPortGetCoreID());
/*    Serial.print("Gyroscope 1 ");
    Serial.print("X: ");
    Serial.print(XG1, 1);
    Serial.print(" rps, ");
    Serial.print("Y: ");
    Serial.print(YG1, 1);
    Serial.print(" rps, ");
    Serial.print("Z: ");
    Serial.print(ZG1, 1);
    Serial.println(" rps");*/
    delay(100);
}
void MPU2(){
    sensors_event_t a, g, temp;
    mpu2.getEvent(&a, &g, &temp);
    XG2 = g.gyro.x;
    YG2 = g.gyro.y;
    ZG2 = g.gyro.z;
    //diagnostic Mode
  /*  Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());
    Serial.print("Gyroscope 2 ");
    Serial.print("X: ");
    Serial.print(XG1, 1);
    Serial.print(" rps, ");
    Serial.print("Y: ");
    Serial.print(YG1, 1);
    Serial.print(" rps, ");
    Serial.print("Z: ");
    Serial.print(ZG1, 1);
    Serial.println(" rps");*/
    delay(100);
}
