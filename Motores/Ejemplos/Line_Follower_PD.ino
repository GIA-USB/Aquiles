#include <QTRSensors.h>  // Pololu QTR Library 

//line sensor defines
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2


//line sensor declarations
// sensors 0 through 7 are connected to digital pins 2 through 9, respectively
QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7, 8, 9},
		   NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

int pwm_a = 10;  //PWM control for motor outputs 1 and 2 is on digital pin 10
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13
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
int motorspeed=0;


void setup()
{
    pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
    pinMode(pwm_b, OUTPUT);
    pinMode(dir_a, OUTPUT);
    pinMode(dir_b, OUTPUT);

    analogWrite(pwm_a, 0);  //set both motors to run at (100/255 = 39)% duty cycle (slow)
    analogWrite(pwm_b, 0);
  

  
    Serial.begin(115200);   
  
  
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
 
    // hardware line follow mode 
    if (mode > 500)
    {
	follow_line(line_position);
    }
    else
    { //not hard wired line following

    
	if (Serial.available() > 0) {
// read the incoming byte:
	    incomingByte = Serial.read();
	}



	Serial.println("*");

  
	switch(incomingByte)
	{
	case 's':
	    digitalWrite(dir_a, LOW);  
	    analogWrite(pwm_a, 0);
	    digitalWrite(dir_b, LOW);  
	    analogWrite(pwm_b, 0);
	    Serial.println("Stop\n");
	    incomingByte='*';
      
	    break;
     
	case 'f':
	    digitalWrite(dir_a, LOW);  
	    analogWrite(pwm_a, 255);
	    digitalWrite(dir_b, LOW);  
	    analogWrite(pwm_b, 255);
	    Serial.println("Forward\n");
	    incomingByte='*';
	    break;
    
	case 'b':
	    digitalWrite(dir_a, HIGH);  
	    analogWrite(pwm_a, 255);
	    digitalWrite(dir_b, HIGH);  
	    analogWrite(pwm_b, 255);
	    Serial.println("Backward\n");
	    incomingByte='*';
	    break;
     
	case 'l':
	    digitalWrite(dir_a, LOW); 
	    analogWrite(pwm_a, 255);
	    digitalWrite(dir_b, HIGH);  
	    analogWrite(pwm_b, 255);
	    Serial.println("Rotate Left\n");
	    incomingByte='*';
	    break;

     
	case 'r':
	    digitalWrite(dir_a, HIGH); 
	    analogWrite(pwm_a, 255);
	    digitalWrite(dir_b, LOW);  
	    analogWrite(pwm_b, 255);
	    Serial.println("Rotate Right\n");
	    incomingByte='*';
	    break;
     
	case 'g': //serial follow the line command
     
	    follow_line(line_position);
     
     
	    Serial.print("::");
	    Serial.print(error);
	    Serial.print('\t');
	    Serial.print(PV);
	    Serial.print('\t');
	    Serial.print(m1Speed);
	    Serial.print('\t');
	    Serial.print(m2Speed);
	    Serial.print('\t');
	    Serial.print(lastError);
	    Serial.print('\t');

	    break;
     
      
	case 'v':
	    Serial.println("Version 04302013a - PD");
	    incomingByte='*';
	    break;
     
	case 'h':
	    Serial.println("(f)orward");
	    Serial.println("(b)ackward");
	    Serial.println("(l)eft");
	    Serial.println("(r)ight");
	    Serial.println("(s)top");
	    Serial.println("(g)follow line");
	    Serial.println("(v)erson");
	    Serial.println("(h)elp");
	    delay(1000);
	    incomingByte='*';
	    break;
     
   
	}  // end command switch
  
    }//end not hardwired line following            
  
    delay(100);


}  // end loop


//line following subroutine
// PD Control


void follow_line(int line_position)  //follow the line

{

    Serial.println("Follow the line (hardware)");

 
    switch(line_position)
    {
    case 0:

  
	digitalWrite(dir_a, LOW); 
	analogWrite(pwm_a, 200);
	digitalWrite(dir_b, HIGH);  
	analogWrite(pwm_b, 200);
	Serial.println("Rotate Left\n");
	break;

    case 7000:
	digitalWrite(dir_a, HIGH); 
	analogWrite(pwm_a, 200);
	digitalWrite(dir_b, LOW);  
	analogWrite(pwm_b, 200);
	Serial.println("Rotate Right\n");
	break;
 
    default:
	error = (float)line_position - 3500;
 
	// set the motor speed based on proportional and derivative PID terms
	// kp is the a floating-point proportional constant 
	//(maybe start with a value around 0.5)
	// kd is the floating-point derivative constant 
	//(maybe start with a value around 1)
	//note that when doing PID, it's very important you get your signs right,
	//or else the
	// control loop will be unstable
	kp=.5;
	kd=1;
   
	PV = kp * error + kd * (error - lastError);
	lastError = error;
  
	//this codes limits the PV (motor speed pwm value)  
	// limit PV to 55
	if (PV > 55)
	{
	    PV = 55;
	}
  
	if (PV < -55)
	{
	    PV = -55;
	}
    
  
	m1Speed = 170 + PV;
	m2Speed = 170 - PV;
     
	//set motor speeds
       
	digitalWrite(dir_a, LOW);  
	analogWrite(pwm_a, m2Speed);
	digitalWrite(dir_b, LOW);  
	analogWrite(pwm_b, m1Speed);
	break;
    }
} // end follow line  
  
     



