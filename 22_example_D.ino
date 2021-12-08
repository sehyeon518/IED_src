/////////////////////////////////////////////////////////////////
/////////////////////////// variables ///////////////////////////
/////////////////////////////////////////////////////////////////
#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255 // mm
#define _DIST_MIN 100    // mm
#define _DIST_MAX 410    // mm

#define _DIST_ALPHA 0.3

#define _DUTY_MIN 553
#define _DUTY_NEU 1476
#define _DUTY_MAX 2399

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 100 //30

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define _KP 0.0
#define _KD 40

Servo myservo;

float dist_target; // 255 mm
float dist_raw, dist_ema, dist_ema_prev, dist_cali;

float alpha;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;  // interval 당 최대 변화량
int duty_target, duty_curr, duty_neutral;

float error_curr, error_prev, control, pterm, dterm, iterm;

int count;

/////////////////////////////////////////////////////////////////
///////////////////////////// setup /////////////////////////////
/////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  Serial.begin(57600);

  myservo.attach(PIN_SERVO);

  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
  duty_neutral = _DUTY_NEU + 40;
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  
  dist_target = _DIST_TARGET;
  dist_raw = dist_ema = dist_ema_prev = dist_cali = 0;
  alpha = _DIST_ALPHA;
  count = 0;
  
  myservo.writeMicroseconds(duty_neutral);
  duty_curr = duty_target = duty_neutral;
}


/////////////////////////////////////////////////////////////////
///////////////////////////// loop //////////////////////////////
/////////////////////////////////////////////////////////////////
void loop() {
  if (millis() < last_sampling_time_dist + _INTERVAL_DIST) return;
  else event_dist = true;
  if (event_dist) {
    event_dist = false;
    dist_raw = ir_distance();
    dist_cali = ir_distance_filtered();
    if (count == 1) dist_ema = 0;
    else {
      dist_ema = alpha * dist_cali + (1 - alpha) * dist_ema_prev;
      dist_ema_prev = dist_ema;
    }
    if(dist_ema > 200 && dist_ema < 350) digitalWrite(PIN_LED, 0);
    else digitalWrite(PIN_LED, 255);
  
    error_curr = dist_target - dist_ema;
    pterm = error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = dterm;
    
    duty_target = duty_neutral + control;

    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    error_prev = error_curr;
  }
  
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
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    if (dist_ema < 100) {
      myservo.writeMicroseconds(duty_neutral - 20);
    }
    else if (dist_cali < _DIST_MIN || dist_cali > _DIST_MAX) {
      dist_cali = dist_ema_prev;
    }
  }
}

/////////////////////////////////////////////////////////////////
////////////////////////// ir_distance //////////////////////////
/////////////////////////////////////////////////////////////////
float ir_distance(void){
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
float ir_distance_filtered(void) {
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return (16*val + 700)/23;
}
