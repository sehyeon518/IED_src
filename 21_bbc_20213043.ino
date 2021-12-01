#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

//global variables
Servo myservo;
#define _DUTY_NEU 1476

int a, b; // unit : mm

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  Serial.begin(57600);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  a = 86;
  b = 306;
}

float ir_distance(void){
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

int sm_degree(float distance){
  if (distance >= 255.0){
    return 700;
  }
  else{
    return 2000;
  }
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist < 224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);

  int degree = sm_degree(raw_dist);
  myservo.writeMicroseconds(degree);
  
  delay(20);
}
