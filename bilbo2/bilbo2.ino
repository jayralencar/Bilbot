#include <QTRSensors.h>
#include <AFMotor.h>

#define QTD_SENSORES  7

int sensores[QTD_SENSORES] = {8,9,10,11,12,13,14};

AF_DCMotor direito(1); 
AF_DCMotor esquerdo(2); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int Kp =  50;
int Ki = 25;
int Kd = 25; 
int last_kp = 0;
int  lastValue= 0;
int ki=0;

void loop() {
  // put your main code here, to run repeatedly:
 int position = readLine();
  
  float kp = position - 3000;
//    kp = kp/2;
  Serial.print(kp);
  float kd = kp - last_kp;
  ki += kp;

  // Remembering the last position
  last_kp = kp;

//  float power_difference = kp / 8 + ki / 10000 + kd * 4 / 2;
  float power_difference = kp / 21 + ki / 10000 + kd * 1.5 / 2;
//  float power_difference = kp / 20  + ki / 10000 + kd * 3/2; // Original
//  float power_difference = kp / 2 + ki / 20000 + kd * 4 / 3; //com 5 sensores funcionou bem
  
  
  Serial.print("\t");
  Serial.println(power_difference);
  const int max_speed = 120;

  if (power_difference > max_speed) power_difference = max_speed;
  if (power_difference < -max_speed) power_difference = -max_speed;
  if (power_difference < 0){

    motorMove(max_speed + power_difference, max_speed);
  }
  else{
    motorMove(max_speed, max_speed - power_difference);
  }
}

void motorMove(int left, int rigth){
  direito.setSpeed(rigth > 0 ? rigth : (rigth * -1));
  esquerdo.setSpeed(left > 0 ? left : (left * -1));
  direito.run(rigth > 0 ? FORWARD : BACKWARD);
  esquerdo.run(left > 0 ? FORWARD : BACKWARD);
}


int readLine(){
  int sum;
  long avg;
  bool online = false;
  for(int i = 0; i < QTD_SENSORES  ; i++){
     int value = 1000 - analogRead(sensores[i]);
//     Serial.print(value);
//     Serial.print("\t");
     if(value > 500){
        online = true;
     }
     if(value > 200){
        avg += (long)(value) * (i*1000); 
        sum += value;
     }
  }

  if(!online){
        // If it last read to the left of center, return 0.
        if(lastValue < (QTD_SENSORES-1)*1000/2)
            return 0;

        // If it last read to the right of center, return the max.
        else
            return (QTD_SENSORES-1)*1000;
  }

  lastValue = avg/sum;
  return lastValue;
 
}
