#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.1

#define _DUTY_MIN 553
#define _DUTY_NEU 1476
#define _DUTY_MAX 2399

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define _KP 0.1

Servo myservo;

float dist_target;
float dist_raw, dist_ema, dist_cali;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr;

float error_curr, error_prev, control, pterm, dterm, iterm;


int a, b; // unit : mm

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  Serial.begin(57600);

  myservo.attach(PIN_SERVO);
  
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
  return 1530 + (300.0 - distance);
//  if (distance <= 300.0){
//    return 1520 + (255.0 - distance);
//  }
//  else{
//    return 1520 - (distance - 255.0);
//  }
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

  int degree = sm_degree(dist_cali);
  myservo.writeMicroseconds(degree);
  
  delay(20);
}
