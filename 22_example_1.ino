#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DUTY_MIN 553
#define _DUTY_NEU 1476
#define _DUTY_MAX 2399

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

Servo myservo;

float dist_raw, dist_ema, dist_cali;


void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  Serial.begin(57600);

  myservo.attach(PIN_SERVO);
}

float ir_distance(void){
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

int sm_degree(float distance){
  return 1530 + (245.0 - distance)*0.5;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = (16*raw_dist + 700)/23;
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist < 224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);

  int degree = sm_degree(dist_cali);
  myservo.writeMicroseconds(degree);
  
  delay(20);
}
