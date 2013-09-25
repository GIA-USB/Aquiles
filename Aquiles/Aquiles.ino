#include <QTRSensors.h>

//Motor Definitions
#define LMotorP1 8
#define LMotorP2 9
#define RMotorP1 12
#define RMotorP2 13

//L293D Enable Pins
#define EnablePL 10
#define EnablePR 11

//Motor Speed
#define Speed 50

//Motor Maximun Speed
#define LM_Max_Speed 255
#define RM_Max_Speed 255

//Delays
#define MotorDelay 0
#define ReadDelay 50
#define DebugMotorSec 3000 //What is this?

//Light Sensor Definitions
#define NUM_SENSORS  6
#define TIMEOUT      2500
#define EMITTER_PIN  1

QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7},
				   NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

boolean GoneR, GoneL, GotLost, GoingF, Intersection, Center, Angle;
int SensorCount;
int MotorSpeed = Speed;
int SensorID;

void setup() {
  
        pinMode(EnablePL, OUTPUT);
        pinMode(EnablePR, OUTPUT);
        
        pinMode(LMotorP1, OUTPUT);
        pinMode(LMotorP2, OUTPUT);
        pinMode(RMotorP1, OUTPUT);
        pinMode(RMotorP2, OUTPUT);
  
	delay(500);
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);		// turn on Arduino's LED to indicate we are in calibration mode
	for (int i = 0; i < 400; i++)	// make the calibration take about 10 seconds
	{
		qtrrc.calibrate();	// reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
	}
	digitalWrite(13, LOW);		// turn off Arduino's LED to indicate we are through with calibration

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

void stopMotors(int Motor) {
	delay(MotorDelay);
	if (Motor == 1) {
		digitalWrite(LMotorP1, LOW);
		digitalWrite(LMotorP2, LOW);
	}
	else if (Motor == 2) {
		digitalWrite(RMotorP1, LOW);
		digitalWrite(RMotorP2, LOW);
	}
	delay(MotorDelay);
}


void runMotors(int Motor) {
	delay(MotorDelay);
	if (Motor == 1) {
		digitalWrite(LMotorP1, HIGH);
		digitalWrite(LMotorP2, LOW);
	}
	else if (Motor == 2) {
		digitalWrite(RMotorP1, LOW);
		digitalWrite(RMotorP2, HIGH);
	}
	delay(MotorDelay);
}

void reverseMotors(int Motor){
	delay(MotorDelay);
	if (Motor == 1) {
		digitalWrite(LMotorP1, LOW);
		digitalWrite(LMotorP2, HIGH);
	}
	else if (Motor == 2) {
		digitalWrite(RMotorP1, HIGH);
		digitalWrite(RMotorP2, LOW);
	}
	delay(MotorDelay);
}

void motorSpeed(int RMotorSpeed, int LMotorSpeed) {
	if (LMotorSpeed > LM_Max_Speed) LMotorSpeed = LM_Max_Speed;		// limit top speed
	if (RMotorSpeed > RM_Max_Speed) RMotorSpeed = RM_Max_Speed;		// limit top speed
	if (LMotorSpeed < 0) LMotorSpeed = 0;					// keep motor above 0
	if (RMotorSpeed < 0) RMotorSpeed = 0;					// keep motor speed above 0
	analogWrite(EnablePL, LMotorSpeed);
	analogWrite(EnablePR, RMotorSpeed);
}

boolean fast (unsigned int position = qtrrc.readLine(sensorValues)) {
  
  unsigned int PosOne = position;
  Serial.print(PosOne);
  Serial.print(" ");
  
  delay(ReadDelay);
  
  position = qtrrc.readLine(sensorValues);
  unsigned int PosTwo = position;
  Serial.print(PosTwo);
  Serial.print(" ");
  
  Serial.print(abs(PosOne - PosTwo));
  Serial.print(" ");
  
  if ( abs(PosOne - PosTwo) > 3000) {
    return true; 
  } else {
    return false;
  }

}

boolean lost (int sensor[]) {
   
   boolean GotLost = true;
   
   for (unsigned int i = 0; i < NUM_SENSORS; ++i) {
      GotLost = GotLost && ((sensor[i] * 10 / 1001) < 7);
   }
   
   Serial.print(GotLost);
   Serial.print(" ");
   
   delay(ReadDelay);
   
   return GotLost;
}

void loop() {

  unsigned int position = qtrrc.readLine(sensorValues);
  
  int HighSpeed = MotorSpeed * 2;
  int NormalSpeed = MotorSpeed * 1.7;
  int LowSpeed = MotorSpeed / 2;
  int CurveSpeed = MotorSpeed / 5;
  
  GotLost = true;
  SensorCount = 0;
  Intersection = false;
  Angle = false;
  
GoneR = position > 2700;
GoneL = position < 2300;
  
   for (unsigned int i = 0; i < NUM_SENSORS; ++i) {
      GotLost = GotLost && ((sensorValues[i] * 10 / 1001) < 7);
   }

if (!GotLost) {
  runMotors(1);
  runMotors(2);
  if (GoneR) {
    motorSpeed (HighSpeed, NormalSpeed);
  } else if (GoneL) {
    motorSpeed (NormalSpeed, HighSpeed);
  } else if (!GoneR && !GoneL) {
    motorSpeed (HighSpeed, HighSpeed);
  }
}

if (GotLost) {
  if (GoneR) {
    reverseMotors(1);
    runMotors(2);
    motorSpeed (HighSpeed, CurveSpeed);
  } else if (GoneL) {
    runMotors(1);
    reverseMotors(2);
    motorSpeed (CurveSpeed, HixghSpeed);
  }
}

}

