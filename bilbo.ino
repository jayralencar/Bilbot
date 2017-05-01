#include <QTRSensors.h>
#include <AFMotor.h>

AF_DCMotor direito(1); 
AF_DCMotor esquerdo(2); 

#define QUANT_SENSORS             5  /* Defined amount of sensors. */
#define QUANT_SAMPLES_PER_SENSOR  2  /* sample mean by analog readings of each sensor */
#define sensorLinePin0 8
#define sensorLinePin1 9
#define sensorLinePin2 10
#define sensorLinePin3 11
#define sensorLinePin4 12


QTRSensorsAnalog qtra((unsigned char[]) {
  sensorLinePin0, sensorLinePin1, sensorLinePin2, sensorLinePin3, sensorLinePin4
}, QUANT_SENSORS, QUANT_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[QUANT_SENSORS];

float last_kp;
float ki;
bool motorLigado = false;
int lastError = 0;

int sensores[QUANT_SENSORS] = {8,9,10,11,12};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(21, OUTPUT);
  calibration();
  printCalibration();
}

void pid1(){
  // put your main code here, to run repeatedly:
  unsigned int position = qtra.readLine(sensorValues);

  
  float kp = ((int)position) - 2000;
  kp = kp/2;
  Serial.print(kp);
  float kd = kp - last_kp;
  ki += kp;

  // Remembering the last position
  last_kp = kp;

//  float power_difference = kp / 10 + ki / 10000 + kd * 3 / 2;
  float power_difference = kp / 2 + ki / 20000 + kd * 4 / 3;
  Serial.print("\t");
  Serial.println(power_difference);
  const int max_speed = 100;

  if (power_difference > max_speed) power_difference = max_speed;
  if (power_difference < -max_speed) power_difference = -max_speed;
  if (power_difference < 0)
    motorMove(max_speed + power_difference, max_speed);
  else
    motorMove(max_speed, max_speed - power_difference);  
}

void loop() {
 pid1(); 
}
int Kp =  50;
int Ki = 25;
int Kd = 25;  

void pid3(){
  unsigned int position = qtra.readLine(sensorValues);

  
  float error = ((int)position) - 2000;
  error  = error /2;
  
  int power_difference = Kp * error + Kd * (error - lastError);
  lastError = error;

    const int max_speed = 255;

  if (power_difference > max_speed) power_difference = max_speed;
  if (power_difference < -max_speed) power_difference = -max_speed;
  if (power_difference < 0)
    motorMove(max_speed + power_difference, max_speed);
  else
    motorMove(max_speed, max_speed - power_difference);  
  
  
}

void pid2(){
  int position = qtra.readLine(sensorValues);
//  int error = position;
  int error = (position - 2000)/1000;
  Serial.print(error);
  Serial.print("\t");

  float P = error;
  float I = I + error;
  float D = error - lastError;
  lastError = error;
  float power_difference = (Kp*P) + (Ki*I) + (Kd*D);
  Serial.println(power_difference);

  const int max_speed = 80;

  if (power_difference > max_speed) power_difference = max_speed;
  if (power_difference < -max_speed) power_difference = -max_speed;
  if (power_difference < 0)
    motorMove(max_speed + power_difference, max_speed);
  else
    motorMove(max_speed, max_speed - power_difference);  
}

void calibration(){
  digitalWrite(21,1);
  motorMove(100, -100);
  for(int i = 0 ; i < 80; i++){
    qtra.calibrate();  
  }
//  delay(20);
  motorStop();
  for (int i = 0; i < QUANT_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(2000);
  digitalWrite(21,0);
  
}

void motorMove(int left, int rigth){
  direito.setSpeed(rigth > 0 ? rigth : (rigth * -1));
  esquerdo.setSpeed(left > 0 ? left : (left * -1));
  direito.run(rigth > 0 ? FORWARD : BACKWARD);
  esquerdo.run(left > 0 ? FORWARD : BACKWARD);
}

void motorStop(){
  direito.run(RELEASE);
  esquerdo.run(RELEASE);  
}

void printCalibration() {

  Serial.println(">>>>>| Calibração de Sensores IR. |<<<<<");

  /* Displays MIN valore obtained. */
  for (int i = 0; i < QUANT_SENSORS; i++) {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  /* Displays MAX obitidos values. */
  for (int i = 0; i < QUANT_SENSORS; i++) {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println("---------------------------------------");
}

int readLine(){
  int exemples = 1;
  int sv[QUANT_SENSORS];
  int sum;
  for(int j = 0; j < 4; j++){
    for(int i = 0; i < 5; i++){
        sum += analogRead(sensores[i]) > 500 ? 0 : i*1;
    }
  }
  return sum/4 - 2;
}

