#include <QTRSensors.h>  // Pololu QTR Library 

//line sensor defines
#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   1     // emitter is controlled by digital pin 2

#define motordelay 0

#define M1_MAX_SPEED 255
#define M2_MAX_SPEED 255


/////////////////////////////////////////////
//MOTOR 1 ES EL DERECHO Y EL 2 EL IZQUIERDO//
/////////////////////////////////////////////


//line sensor declarations
// sensors 0 through 7 are connected to digital pins 2 through 9, respectively
QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7},
NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

int enablePin1 = 10;  //PWM control for motor outputs 1 and 2 is on digital pin 10
int enablePin2 = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int motor1pole1 = 8;  //direction control for motor outputs 1 and 2 is on digital pin 12
int motor1pole2 = 9;
int motor2pole1 = 12;  //direction control for motor outputs 3 and 4 is on digital pin 13
int motor2pole2 = 13;
int incomingByte = 0;	// for incoming serial data
int encoderCount=0;
int encoderOld=0;
int encoderState=0;
int mode=0;
int sensorState=0;
int temp_var=0;


// pid loop vars
float error=0;
float lastError=0;
float PV =0 ;
float kp = 0;
float ki = 0;
float kd =0;
int m1Speed=0;
int m2Speed=0;
int line_position=0;


void setup()
{
  pinMode(motor1pole1, OUTPUT);  //Set control pins to be outputs
  pinMode(motor1pole2, OUTPUT);
  pinMode(motor2pole1, OUTPUT);
  pinMode(motor2pole2, OUTPUT);

  analogWrite(enablePin1, OUTPUT);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
  analogWrite(enablePin2, OUTPUT);
  
  motorspeed(0,0);

  
  Serial.begin(9600);   
  
  
  Serial.println("Zagros Robotics, Inc.");
  Serial.println("Magician Line Follower Demo PC Control\n ");
  Serial.println("Calibrating sensor");
  // calibrate line sensor
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  
  // print calibration results
  
  // print the calibration minimum values measured when emitters were on
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
  Serial.println("Calibration Complete");
  delay(1000);
  
}

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
// read line follow mode

mode = analogRead(0);  
//Serial.print("Line Follower Mode:");
//Serial.println(mode);   

 
  // Serial.print("Zagros Robotics, Inc.");
  // line sensor values
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int line_position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
 //   Serial.print(sensorValues[i]);
 //   Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(line_position); // comment this line out if you are using raw values
  
  //delay(250);

  follow_line(line_position);

  delay(100);
}

//line following subroutine
// PD Control


void follow_line(int line_position)  //follow the line

{

Serial.println("Follow the line (hardware)");

 
switch(line_position)
{
 case 0:

  
       motorspeed(150,100);
       motorforward(2);
       motorback(1);
       Serial.println("Rotate Left\n");
break;

case 5000:

       motorspeed(100,150);
       motorforward(1);
       motorback(2);
       Serial.println("Rotate Right\n");
break;
 
 default:
   error = (float)line_position - 2500;
 
  // set the motor speed based on proportional and derivative PID terms
  // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
  // kd is the floating-point derivative constant (maybe start with a value around 1)
  // note that when doing PID, it's very important you get your signs right, or else the
  // control loop will be unstable
  kp=.5;
  kd=1;
   
  PV = kp * error + kd * (error - lastError);
  lastError = error;
  
  //this codes limits the PV (motor speed pwm value)  
  // limit PV to 55
  if (PV > 45)
  {
    PV = 45;
  }
  
  if (PV < -45)
  {
    PV = -45;
  }
    
  
  m1Speed = 170 + PV;
  m2Speed = 170 - PV;
     
      //set motor speeds
       
       motorspeed(m1Speed, m2Speed);
       motorforward(1);
       motorforward(2);
       /*digitalWrite(dir_a, LOW);  
       analogWrite(pwm_a, m2Speed);
       digitalWrite(dir_b, LOW);  
       analogWrite(pwm_b, m1Speed);*/
    break;
}
} // end follow line  
  
     



