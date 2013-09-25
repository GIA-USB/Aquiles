#include <QTRSensors.h>

// Definiciones para los motores ///

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



//////////////////////////////////////////////////////////////////////////////

boolean seFueIzq = false;
boolean seFueDer = false; 
int veloIzq;
int veloDer;

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
motorspeed(255, 255);
//  veloIzq = 255;
//  veloDer = 255;
//  
  motorforward(1);
  motorforward(2);
  
  delay(50);
  
}
    
