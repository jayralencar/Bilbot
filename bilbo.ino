#include <QTRSensors.h>
#include <AFMotor.h>

AF_DCMotor direito(1); 
AF_DCMotor esquerdo(2); 

#define QUANT_SENSORS             5  /* Defined amount of sensors. */
#define QUANT_SAMPLES_PER_SENSOR  4  /* sample mean by analog readings of each sensor */
#define sensorLinePin0 8
#define sensorLinePin1 9
#define sensorLinePin2 10
#define sensorLinePin3 11
#define sensorLinePin4 12

#define Kp 0.2
#define Kd 5

QTRSensorsAnalog qtra((unsigned char[]) {
  sensorLinePin0, sensorLinePin1, sensorLinePin2, sensorLinePin3, sensorLinePin4
}, QUANT_SENSORS, QUANT_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[QUANT_SENSORS];

float last_kp;
float ki;
bool motorLigado = false;
int lastError = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(21, OUTPUT);
  calibration();
  printCalibration();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned int position = qtra.readLine(sensorValues);

  int error = position - 2000;
  Serial.print(error);
  Serial.print("\t");
  int power_difference = Kp * error + Kd * (error - lastError);
  lastError = error;

  Serial.println(power_difference);
  

  const int max_speed = 180;

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
  for(int i = 0 ; i < 100; i++){
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
