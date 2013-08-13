#include <QTRSensors.h>

// Definiciones para los motores ////////////////////

// map motor poles to Arduino pins
#define motor1pole1 9 
#define motor1pole2 8
#define motor2pole1 12
#define motor2pole2 13

// map L293d motor enable pins to Arduino pins
#define enablePin1 11
#define enablePin2 10

#define M1_MAX_SPEED 100
#define M2_MAX_SPEED 100

#define motordelay 30
#define debugmotorsec 3000

///////////////////////////////////////////////////

// Definiciones para el sensor de Luz //////////////////////////////////////////////////////////

#define NUM_SENSORS             6  // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN             1  // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

///////////////////////////////////////////////////////////////////////////////////////////////

boolean seFueIzq = false;
boolean seFueDer = false; 

void setup() {
  // Motores
  Serial.begin(9600);

  // set mapped L293D motor1 and motor 2 enable pins on Arduino to output (to turn on/off motor1 and motor2 via L293D)
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // set mapped motor poles to Arduino pins (via L293D)
  pinMode( motor1pole1, OUTPUT);
  pinMode( motor1pole2, OUTPUT);
  pinMode( motor2pole1, OUTPUT);
  pinMode( motor2pole2, OUTPUT);
  motorspeed(0, 0);
  
  // Sensor de luz
  
    delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  
}

int mspeed = 255;  // pick a starting speed up to 255

// MOTOR FUNCTIONS

void motorstop(int motornum){
  delay(motordelay);
  if (motornum == 1) {
    digitalWrite(motor1pole1, LOW);
    digitalWrite(motor1pole2, LOW);
  }
  else if (motornum == 2) {

    digitalWrite(motor2pole1, LOW);
    digitalWrite(motor2pole2, LOW);
  }
  delay(motordelay);
}

void motorforward(int motornum){
  if (motornum == 1) {
    digitalWrite(motor1pole1, HIGH);
    digitalWrite(motor1pole2, LOW);
  }
  else if (motornum == 2) {

    digitalWrite(motor2pole1, LOW);
    digitalWrite(motor2pole2, HIGH);
  }
  delay(motordelay);
}

void motorback(int motornum){
  if (motornum == 1) {
    digitalWrite(motor1pole1, LOW);
    digitalWrite(motor1pole2, HIGH);
  } 
  else if (motornum == 2) {
    digitalWrite(motor2pole1, HIGH);
    digitalWrite(motor2pole2, LOW);
  }
  delay(motordelay);
}

void motorspeed(int motor1speed, int motor2speed) {
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor1speed < 0) motor1speed = 0; // keep motor above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0
  analogWrite(enablePin1, motor1speed);
  analogWrite(enablePin2, motor2speed);
}


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues);
  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i] * 10 / 1001);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values
  position = position - 2500;
  
  //delay(250);
  
  motorspeed(255, 255);
  
  seFueIzq = position > 450;
  seFueDer = position < -450;
  
  if (!seFueIzq && !seFueDer) {
    motorforward(1);
    motorforward(2);
  }
  
  else if (seFueDer) {
    motorstop(1);
    motorforward(2); 
  }
  
  else if (seFueIzq){
    motorstop(2);
    motorforward(1);
  }
  
  delay(10);
  
}
