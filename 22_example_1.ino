/////////////////////////////////////////////////////////////////
/////////////////////////// variables ///////////////////////////
/////////////////////////////////////////////////////////////////
#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.2

#define _DUTY_MIN 553
#define _DUTY_NEU 1476
#define _DUTY_MAX 2399

#define _SERVO_ANGLE 17
#define _SERVO_SPEED 50

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define _KP 1.5
#define _KD 0.0

Servo myservo;

float dist_target;
float dist_raw, dist_ema, dist_ema_prev, dist_cali;

float alpha;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr, duty_neutral;

float error_curr, error_prev, control, pterm, dterm, iterm;

int a, b;
int count;

/////////////////////////////////////////////////////////////////
///////////////////////////// setup /////////////////////////////
/////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  Serial.begin(57600);

  myservo.attach(PIN_SERVO);

  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / _SERVO_ANGLE * _INTERVAL_SERVO / 1000;
  duty_curr = duty_target = duty_neutral = _DUTY_NEU + 60;
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  dist_target = _DIST_TARGET;
  dist_raw = dist_ema = dist_ema_prev = dist_cali = 0;
  alpha = _DIST_ALPHA;

  a = 80;
  b = 330;
  
  count = 0;

  myservo.writeMicroseconds(duty_neutral);
}

/////////////////////////////////////////////////////////////////
///////////////////////////// loop //////////////////////////////
/////////////////////////////////////////////////////////////////
void loop() {
  count += 1; 
/////////////////////////// distance ////////////////////////////
  if (millis() < last_sampling_time_dist + _INTERVAL_DIST) return;
  else event_dist = true;
  if (event_dist) {
    event_dist = false;
//    dist_raw = ir_distance();
//    dist_cali = ir_distance_filtered(dist_raw);
    float volt = float(analogRead(PIN_IR));
    dist_raw = ((6762.0/(volt-9.0))-4.0) * 10.0;
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
    if (dist_cali < _DIST_MIN || dist_cali > _DIST_MAX) {
      dist_cali = dist_ema_prev;
    }
    if (count == 1) dist_ema = 0;
    else {
      dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;
      dist_ema_prev = dist_ema;
    }
    ///
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm;
    duty_target = duty_neutral + control;
    ///
    error_prev = error_curr;  
    if(dist_ema > 200 && dist_ema < 350) digitalWrite(PIN_LED, 0);
    else digitalWrite(PIN_LED, 255);
  }
  last_sampling_time_dist += _INTERVAL_DIST;
///////////////////////////// servo /////////////////////////////
  if (millis() < last_sampling_time_servo + _INTERVAL_SERVO) return;
  else event_servo = true;
  if (event_servo) {
    event_servo = false;
    if (duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }
    myservo.writeMicroseconds(duty_curr);
  }  
  last_sampling_time_servo += _INTERVAL_SERVO;
///////////////////////////// serial ////////////////////////////
  if (millis() < last_sampling_time_serial + _INTERVAL_SERIAL) return;
  else event_serial = true;  
  if (event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",dist_cali:");
    Serial.print(dist_cali);
    Serial.print(",dist_ema:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
  last_sampling_time_serial += _INTERVAL_SERIAL;
/////////////////////////////////////////////////////////////////
}
