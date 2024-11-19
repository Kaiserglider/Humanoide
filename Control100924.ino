//llamar a la libreria del bluetooth
#include "BluetoothSerial.h"
//LLamar a las librerias del modulo PWM
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//llamar a las librerias de los giroscopios
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// Definir modulo PWM
#define PCA9685_ADDR 0x40  // Dirección I2C del PCA9685
//definimos giroscopio y variables
Adafruit_MPU6050 mpu;
int contG = 0;
float angleX = 0, angleY = 0, angleZ = 0; // Acumuladores de ángulos
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0; // Offsets del giroscopio
unsigned long previousTime = 0; // Tiempo anterior para calcular deltaTime
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
int posiciones[14] = {45, 140, 90, 130, 40, 91, 20, 160, 160, 100, 90, 20, 80, 90};
int limitesA[14]= {0, 30, 0, 50, 0, 50, 0, 0, 0, 0, 0, -5, 0, 0};
int limitesB[14]= {130, 180, 130, 180, 150, 180, 180, 180, 195, 180, 180, 180, 180, 180};
int Recto[9]= {40,130,80,100,90,85,90,85};
int C1[8]= {40,110,80,100,90,-5,80,90};
int Ladeado[9]= {60,110,195,100,90,-5,80,90};
int C2[8]= {60,130,195,100,90,85,90,85};
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
int CAM3=8;
//Definir tiempos
int t01 = 50;
int t02 = 10;
int t03 = 35;
int t04 = 15;
int t05 = 25;
int t06 = 100;     // Tiempo de retraso entre movimientos
//int steps = 10;  // Número de pasos para suavizar el movimiento
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
int k03=-15;
int k04=10;
int Length=3;
//Control Brazos
int Dif1[8];
int Dif2[8];
int Add[8];
int Mult = 0;
int contB = 0;
bool equilibrioActivo = false;
float referenciaGiro = 0;  // Puede ser XG1, YG1, o ZG1
int Jam1 = 0;
int Jam2 = 0;
int Jam3 = 0;
int FC = 30 ;
int FG = 15 ;
bool secuenciaActivada = false;


void setup() {
  //Inicialisamos Bluetooth
  Serial.begin(115200); 
  SerialBT.begin(device_name);  // Nombre del dispositivo Bluetooth
  Serial.printf("El dispositivo con nombre \"%s\" está iniciado.\n¡Ahora puedes emparejarlo con Bluetooth!\n", device_name.c_str());
  //esperamos al serial
  while (!Serial) {
       delay(10);// Espera a que el puerto serie esté listo
  }
  
  //inicialisamos PWM y valores iniciales
  pwm.begin();
  pwm.setPWMFreq(330);
  setInitialServoPositions();
  for (int i = 0; i < 9; i++) { //Diagnostic Mode
     Dif1[i]=C1[i]-posiciones[i+6];
     Dif2[i]=C2[i]-posiciones[i+6];
  }
// inicialisamos giroscopio/s
 Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  calibrateGyro(100); // Calibrar giroscopio con 100 muestras

  previousTime = millis();
  //inicialisamos nucleo 0
  xTaskCreatePinnedToCore(
                    Core0,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

//inicialisamos nucleo 1
  xTaskCreatePinnedToCore(
                    Core1,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

//Task1code: check the MPU6050 And control arms
void Core0( void * pvParameters ){
 Serial.print("Task1 started on core ");
    Serial.println(xPortGetCoreID());

    for (;;) {
        contG++;
        contB++;
        // Monitorear el giroscopio
        if (contG > 50 && !equilibrioActivo) {
            MPU1();  // Actualizar valores del giroscopio
            contG = 0;
        }
        // Evaluar si es necesario activar el equilibrio
        if (contB > 50 && !equilibrioActivo) {
            verificarGiroscopio(angleZ);
            verificarGiroscop(angleY);

            contB = 0;
        }
        delay(1);  // Evitar sobrecarga
    }
  } 


//Task2code: check the bluetooth and calculate cinemaatic
void Core1( void * pvParameters ){
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
void smoothMove(int count, int servos[], int startAngles[], int endAngles[], int time, int steps) {
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
 /*     Serial.print("M:");
      Serial.print(servos[j]);
      Serial.write(" ");
      Serial.print("PWM:");
      Serial.print(currentPulse);
      Serial.write(" ");*/
    }
 //   Serial.println();
 //   Serial.print("Task2 running on core ");
 //   Serial.println(xPortGetCoreID());
    delay(time / steps); //Divide el tiempo por los pasos para suavizar
  }
}
void fastMove(int count, int servos[], int startAngles[], int endAngles[], int time) {
  //Array de Pulsos Iniciales y Finales
  int pulsesEnd[count];

  //Calcula el pulso inicial, final y paso para cada servo
  for (int i = 0; i < count; i++) {
    pulsesEnd[i] = angleToPulse(endAngles[i]);
  }
  //Mueve los servos en pasos
    for (int j = 0; j < count; j++) {
      pwm.setPWM(servos[j], 0, pulsesEnd[j]);/*
      Serial.print("M:");
      Serial.print(servos[j]);
      Serial.write(" ");
      Serial.print("PWM:");
      Serial.print(pulsesEnd[j]);
      Serial.write(" ");*/
    }
//    Serial.println();
 //   Serial.print("Task2 running on core ");
 //   Serial.println(xPortGetCoreID());
    delay(time); 
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

//  Serial.print(X0); //Diagnostic Mode
//  Serial.println();
  for (int i = 0; i < CAM; i++) {
    Serial.print(servos[i]);
    Serial.print(" ");
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();
  smoothMove(14, servos, startAngles, AbsAngles, time, steps);
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
  smoothMove(CAM2, servos2, startAngles, AbsAngles, time,steps);
 // Array de Pulsos Iniciales y Finales
    P0 = AbsAngles[0]; P1 = AbsAngles[1]; P2 = AbsAngles[2]; P3 = AbsAngles[3];
    P4 = AbsAngles[4]; P5 = AbsAngles[5];
/*for (int i = 0; i < CAM; i++) { //Diagnostic Mode
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();*/
}
void AdvMoveAbsLF(int time, int X0,int X1,int X2,int X3,int X4,int X5) {
  
  int AbsAngles[] = {
        validarAngulo(0, X0), validarAngulo(1, X1), validarAngulo(2, X2), validarAngulo(3, X3),
        validarAngulo(4, X4), validarAngulo(5, X5)
  };
  int startAngles[]={P0,P1,P2,P3,P4,P5};

//  Serial.print(X0); //Diagnostic Mode
//  Serial.println();
  for (int i = 0; i < CAM2; i++) {
    Serial.print(servos2[i]);
    Serial.print(" ");
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();
  fastMove(CAM2, servos2, startAngles, AbsAngles, time);
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

//  Serial.print(X0); //Diagnostic Mode
//  Serial.println();
  for (int i = 0; i < CAM3; i++) {
    Serial.print(AbsAngles[i]);
    Serial.write("\t");
  }
  Serial.println();
  smoothMove(CAM3, servos3, startAngles, AbsAngles, time, steps);
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
  smoothMove(14, servos, startAngles, RelAngles, time, steps);
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

if (command.startsWith("in1leg")) {
 leg_1();
}
if (command.startsWith("cinematic")) {
  //cinematica
    takeStep(Length, t05);
    delay(t06);
/*    takeStep(Length, t05);
    delay(t06);
    takeStep(Length, t05);
    delay(t06);
    takeStep(Length, t05);
    delay(t06);
    takeStep(Length, t05);*/
}

if (command.startsWith("pwm")) {
  //cinematica
    cinematic2();
    delay(t06);
/*    takeStep(Length, t05);
    delay(t06);
    takeStep(Length, t05);
    delay(t06);
    takeStep(Length, t05);
    delay(t06);
    takeStep(Length, t05);*/
}
if (command.startsWith("prepare")) {
  //cinematica
    initialize();
/*    delay(250);
    XG1 = 0;
    YG1 = 0;
    ZG1 = 0;*/
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

void leg_1 () {
AdvMoveAbs(20,10,45,90,180,150,55,91,20,160,160,100,90,20,80,90);
}

void cinematic2 () {
AdvMoveAbsLF(t04,27,97,54,130,40,91);
delay(t05);
AdvMoveAbsLF(t04,27,97,54,180,115,119);
delay(t05);
AdvMoveAbsLF(t04,25,95,55,180,115,119);
delay(t05);
AdvMoveAbsLF(t04,25,95,55,180,116,121);
delay(t05);
AdvMoveAbsLF(t04,22,93,56,180,116,121);
delay(t05);
AdvMoveAbsLF(t04,22,93,56,180,117,124);
delay(t05);
AdvMoveAbsLF(t04,20,92,57,180,117,124);
delay(t05);
AdvMoveAbsLF(t04,20,92,57,180,118,126);
delay(t05);
AdvMoveAbsLF(t04,17,92,59,180,118,126);
delay(t05);
AdvMoveAbsLF(t04,17,92,59,180,118,129);
delay(t05);
AdvMoveAbsLF(t04,15,91,60,180,118,129);
delay(t05);
AdvMoveAbsLF(t04,15,91,60,179,119,131);
delay(t05);
AdvMoveAbsLF(t04,14,91,62,177,119,131);
delay(t05);
AdvMoveAbsLF(t04,14,91,64,177,119,133);
delay(t05);
AdvMoveAbsLF(t04,12,91,64,175,119,133);
delay(t05);
AdvMoveAbsLF(t04,10,92,66,175,119,135);
delay(t05);
AdvMoveAbsLF(t04,10,92,66,173,118,135);
delay(t05);
AdvMoveAbsLF(t04,9,92,68,173,118,137);
delay(t05);
AdvMoveAbsLF(t04,9,92,68,171,118,137);
delay(t05);
AdvMoveAbsLF(t04,8,93,70,171,118,138);
delay(t05);
AdvMoveAbsLF(t04,8,93,70,168,117,140);
delay(t05);
AdvMoveAbsLF(t04,7,95,73,168,117,140);
delay(t05);
AdvMoveAbsLF(t04,7,95,73,166,116,142);// step PF
delay(t05);
AdvMoveAbsLF(t04,6,97,75,166,116,142);//step middle
delay(t05);
AdvMoveAbsLF(t04,6,97,75,163,115,143);
delay(t05);
AdvMoveAbsLF(t04,0,65,62,163,83,143);
delay(t05);
AdvMoveAbsLF(t04,0,65,62,148,83,127);
delay(t05);
AdvMoveAbsLF(t04,0,64,60,148,85,127);
delay(t05);
AdvMoveAbsLF(t04,0,64,60,150,87,126);
delay(t05);
AdvMoveAbsLF(t04,0,63,57,150,87,126);
delay(t05);
AdvMoveAbsLF(t04,0,63,57,153,88,126);
delay(t05);
AdvMoveAbsLF(t04,0,62,55,153,88,125);
delay(t05);
AdvMoveAbsLF(t04,0,62,55,155,88,125);
delay(t05);
AdvMoveAbsLF(t04,0,62,52,155,88,124);
delay(t05);
AdvMoveAbsLF(t04,0,62,52,158,88,124);
delay(t05);
AdvMoveAbsLF(t04,0,61,50,158,89,122);
delay(t05);
AdvMoveAbsLF(t04,0,61,50,160,89,122);
delay(t05);
AdvMoveAbsLF(t04,0,61,48,160,89,121);// step PF
delay(t05);
AdvMoveAbsLF(t04,0,61,48,161,89,121);
delay(t05);
AdvMoveAbsLF(t04,0,61,46,161,89,119);
delay(t05);
AdvMoveAbsLF(t04,0,61,46,163,89,119);
delay(t05);
AdvMoveAbsLF(t04,2,61,44,163,89,117);
delay(t05);
AdvMoveAbsLF(t04,2,62,44,165,88,117);
delay(t05);
AdvMoveAbsLF(t04,4,62,43,165,88,115);
delay(t05);
AdvMoveAbsLF(t04,4,63,43,166,88,115);
delay(t05);
AdvMoveAbsLF(t04,7,63,41,166,88,113);
delay(t05);
AdvMoveAbsLF(t04,7,64,41,167,87,111);
delay(t05);
AdvMoveAbsLF(t04,9,64,39,167,87,111);
delay(t05);
AdvMoveAbsLF(t04,9,64,39,168,85,108);
delay(t05);
AdvMoveAbsLF(t04,12,65,38,168,85,108);
delay(t05);
AdvMoveAbsLF(t04,12,65,38,169,83,106);
delay(t05);
}
//cinematica

//cinematica
void updateServoPos(int target1, int target2, int target3, char leg){
  if (leg == 'l'){
    AdvMoveAbsLF(t04, P0,P1,P2,posiciones[3]-(target3-90-k01),posiciones[4]+ target2+k02,posiciones[5]+target1+k03);

  }
  else if (leg == 'r'){ 
    AdvMoveAbsLF(t04, posiciones[0]+(target3-90-k01), posiciones[1]-target2-k02,posiciones[2]-target1-k03,P3,P4,P5);
    
  }
}
void updateServoPosB(int target1, int target2, int target3, char leg){
  if (leg == 'l'){
    AdvMoveAbsLF(t04, P0,P1,P2,posiciones[3]-(target3-90-k01),posiciones[4]+ target2+k02,posiciones[5]+target1+k04);

  }
  else if (leg == 'r'){ 
    AdvMoveAbsLF(t04, posiciones[0]+(target3-90-k01), posiciones[1]-target2-k02,posiciones[2]-target1-k04,P3,P4,P5);
    
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

void posB(float x, float z, char leg){
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
  updateServoPosB(hipDeg, kneeDeg, ankleDeg, leg);  
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
    posB(0, i, 'l');
    posB(0, i, 'r');
  }
}
//Editor de variables
void setVariable(String variableName, int newValue) {
  // Actualizar variables individuales
  if (variableName == "T1") {
    t01 = newValue;
//    Serial.print("T1  updated:");
//    Serial.println(t01);
  } else if (variableName == "T2") {
    t02 = newValue;
  } else if (variableName == "T3") {
    t03 = newValue;
  } else if (variableName == "T4") {
    t04 = newValue;
  } else if (variableName == "T5") {
    t05 = newValue;
  } else if (variableName == "T6") {
    t06 = newValue;
  } else if (variableName == "K1") {
    k01 = newValue;
  } else if (variableName == "K2") {
    k02 = newValue;
  } else if (variableName == "K3") {
    k03 = newValue;
  } else if (variableName == "K4") {
    k04 = newValue;  
  } else if (variableName == "FC") {
    FC = newValue;
  } else if (variableName == "l1") {
    l1 = newValue;
  } else if (variableName == "l2") {
    l2 = newValue;
//    Serial.print("L2  updated:");
//    Serial.println(l2);
  } else if (variableName == "stepClearance") {
    stepClearance = newValue;
  } else if (variableName == "stepHeight") {
    stepHeight = newValue;
  } else if (variableName == "Length") {
    Length = newValue;
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
void MPU1() {
   sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Tiempo en segundos
  previousTime = currentTime;

  // Ajustar valores del giroscopio usando los offsets calibrados
  float gyroX = (g.gyro.x - gyroX_offset) * 57.2958; // rad/s a grados/s
  float gyroY = (g.gyro.y - gyroY_offset) * 57.2958;
  float gyroZ = (g.gyro.z - gyroZ_offset) * 57.2958;

  // Acumular los ángulos (integración)
  angleX += gyroX * deltaTime;
  angleY += gyroY * deltaTime;
  angleZ += gyroZ * deltaTime;

  // Filtro complementario: combinar giroscopio con acelerómetro
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 57.2958; // Acelerómetro a grados
  float accelAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958;

  // Combinar ángulos (filtro complementario)
  angleX = 0.98 * (angleX) + 0.02 * accelAngleX;
  angleY = 0.98 * (angleY) + 0.02 * accelAngleY;

  // Imprimir los resultados
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print(" deg, Y: ");
  Serial.print(angleY);
  Serial.print(" deg, Z: ");
  Serial.print(angleZ);
  Serial.println(" deg"); 
}
       //diagnostic Mode
//    Serial.print("Task1 running on core ");
//    Serial.println(xPortGetCoreID());
/*   Serial.print("G1");
    Serial.print(" ");
    Serial.print(g.gyro.x, 6);
    Serial.print(" ");
    Serial.print(g.gyro.y, 6);
    Serial.print(" ");
    Serial.println(g.gyro.z, 6);*/
   // delay(50);

void verificarGiroscopio(float valorReferencia) {
    if (valorReferencia > FC && !Jam1) {
        activarSecuenciaEquilibrio(true);
    } else if (valorReferencia < -FC && !Jam2) {
        activarSecuenciaEquilibrio(false);
    }
    if (FG > valorReferencia && valorReferencia > -FG && !Jam3 && !secuenciaActivada) {
        activarSecuencia();
    }
}

void verificarGiroscop(float valorReferencia) {
    if (valorReferencia < -83) {
    k03 =  -15;
    } else if (valorReferencia > -73) {
    k03 = 10;
    }
}


// Función para activar la secuencia de equilibrio
void activarSecuenciaEquilibrio(bool haciaAdelante) {
    equilibrioActivo = true;  // Marcar que estamos en modo equilibrio

    if (haciaAdelante) {
        /*for (int i = 0; i < 9; i++) {
            Mult = (referenciaGiro * 1) / 10;
            Add[i] = (Dif1[i] * Mult);
        }*/
        Jam1 = 1;
        Jam2 = 0;
        Jam3 = 0;
        AdvMoveAbsA(20, 10, Recto[0], Ladeado[1], Recto[2],
                    Recto[3],Recto[4], Ladeado[5],
                    Ladeado[6], Ladeado[7]);
    } else {
        /*for (int i = 0; i < 9; i++) {
            Mult = (referenciaGiro * 1) / -10;
            Add[i] = (Dif2[i] * Mult);
        }*/
        Jam1 = 0;
        Jam2 = 1;
        Jam3 = 0;
        AdvMoveAbsA(t01, 10, Ladeado[0], Recto[1], Ladeado[2],
                    Ladeado[3], Ladeado[4], Recto[5],
                    Recto[6],Recto[7]);
    }

    // Finalizar el equilibrio después de un pequeño retraso
    delay(50);  // Ajustar según el tiempo de movimiento necesario
    secuenciaActivada = false;
    equilibrioActivo = false;  // Restablecer el estado
}

// Función para activar la secuencia de equilibrio
void activarSecuencia() {
    equilibrioActivo = true;  // Marcar que estamos en modo equilibrio
    secuenciaActivada = true; // Marcar que la secuencia ha sido activada

    Jam1 = 0;
    Jam2 = 0;
    Jam3 = 1;
    AdvMoveAbsA(t01, 10, posiciones[6], posiciones[7], posiciones[8],
                posiciones[9], posiciones[10], posiciones[11],
                posiciones[12], posiciones[13]);

    // Finalizar el equilibrio después de un pequeño retraso
    delay(50);  // Ajustar según el tiempo de movimiento necesario
    equilibrioActivo = false;  // Restablecer el estado
}
void calibrateGyro(int samples) {
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Calibrating gyroscope... Please keep the sensor steady.");
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;

    delay(10); // Pequeña pausa entre mediciones
  }
  gyroX_offset = sumX / samples;
  gyroY_offset = sumY / samples;
  gyroZ_offset = sumZ / samples;

  Serial.println("Calibration complete.");
  Serial.print("Offsets -> X: ");
  Serial.print(gyroX_offset);
  Serial.print(", Y: ");
  Serial.print(gyroY_offset);
  Serial.print(", Z: ");
  Serial.println(gyroZ_offset);
}

