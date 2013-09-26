
// Inclusão de bibliotecas 
#include <QTRSensors.h>

#define NUM_SENSORS  6
#define TIMEOUT      2500
#define EMITTER_PIN  1

QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 5, 6, 7}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

// Definição de variáveis
unsigned int sensors[NUM_SENSORS]; // Matriz para armazenar valores dos sensores

//Motor Definitions
#define LMotorP1 8
#define LMotorP2 9
#define RMotorP1 12
#define RMotorP2 13

//L293D Enable Pins
#define EnablePL 10
#define EnablePR 11

//Motor Maximun Speed
#define LM_Max_Speed 100
#define RM_Max_Speed 100
#define MAX_SPEED    100

//Delays
#define MotorDelay 0
#define ReadDelay 50
#define DebugMotorSec 3000 //What is this?

float volts = 0;
int total = 0;
float average = 0;
int index = 0;
int last_proportional;
int integral;

void motorstop(int Motor) {
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


void motorforward(int Motor) {
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

void motorback(int Motor){
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

void motorspeed(int LMotorSpeed, int RMotorSpeed) {
	if (LMotorSpeed > LM_Max_Speed) LMotorSpeed = LM_Max_Speed;		// limit top speed
	if (RMotorSpeed > RM_Max_Speed) RMotorSpeed = RM_Max_Speed;		// limit top speed
	if (LMotorSpeed < 0) LMotorSpeed = 0;					// keep motor above 0
	if (RMotorSpeed < 0) RMotorSpeed = 0;					// keep motor speed above 0
	analogWrite(EnablePL, LMotorSpeed);
	analogWrite(EnablePR, RMotorSpeed);
}


// Executado na inicialização do Arduino
void setup(){
  Serial.begin(9600);        // Inicializa a comunicação serial
  motorspeed(0,0);
  motorstop(1);
  motorstop(2);
  
  
  /*
  //Autocalibracion
  unsigned int counter; // usado como um simples contador
  for(counter=0; counter<80; counter++){
    if(counter < 20 || counter >= 60){
      motorspeed(100,100);
      motorforward(1);
      motorback(2); //Gira a la derecha
    }
    else{
      motorspeed(100,100);
      motorforward(2);
      motorback(1); //Gira a la izquierda
    }
    // Esta função armazena um conjunto de leituras dos sensores, e mantém
    // informações sobre o máximo e mínimo valores encontrados
    qtrrc.calibrate();
    // Desde que contamos até 80, o total do tempo de calibração
    // será de 80 * 10 = 800 ms
    delay(10);
  }*/
  
  
  
  
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
  
  
  
  motorstop(1);
  motorstop(2); // Garante motores parados após o processo 
                   // de calibração
}


// Esta é a função principal, onde o código inicia. Todo programa Arduino
// deve ter uma função loop() definida em algum lugar
void loop(){

    // Obtém a posição da linha
    // Aqui não estamos interessados nos valores individuais de cada sensor
    delay(25);
    
    unsigned int position = qtrrc.readLine(sensors);
    
    Serial.println(position);
  
  switch (position) {
  
  case 0:

    motorspeed(80,60);
    motorforward(1);
    motorstop(2);
    break;
    
  case 5000:
    motorspeed(60,80);
    motorforward(2);
    motorstop(1);
    break;
    
  default:
  
    // O termo proporcional deve ser 0 quando estamos na linha
    int proportional = ((int)position) - 2500;
    
    // Calcula o termo derivativo (mudança) e o termo integral (soma)
    // da posição
    int derivative = proportional - last_proportional;
    integral += proportional;
    
    // Lembrando a ultima posição
    last_proportional = proportional;
    
    // Calcula a diferença entre o aranjo de potência dos dois motores
    // m1 - m2. Se for um número positivo, o robot irá virar para a 
    // direita. Se for um número negativo, o robot irá virar para a esquerda
    // e a magnetude dos números determinam a agudez com que fará as curvas/giros
    int kp = 0.1; //1/10
    int ki = 1/10000; //1/10000
    int kd = 3/2; //3/2
    int power_difference = proportional*kp + integral*ki + derivative*kd;
    
    // Calcula a configuração atual dos motores.  Nunca vamos configurar
    // um motor com valor negativo
    const int max = 100;
    if(power_difference > max)
      power_difference = max;
    if(power_difference < -max)
      power_difference = -max;
    if(power_difference < 0) 
    //giro a la izquierda
      motorspeed(MAX_SPEED, max+power_difference);
//      set_motors(max+power_difference, max);
    else
    //giro a la derecha
      motorspeed(max-power_difference, MAX_SPEED); 
//      set_motors(max, max-power_difference);
      
    motorforward(1);
    motorforward(2);
    break;
  }
}



