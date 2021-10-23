#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm
#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
// #define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2429 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, dist_ema_prev; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int count;
float alpha;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); // 0 V
  analogWrite(PIN_LED, 255);
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN); // 90 degree

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = dist_ema = dist_ema_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  count = 0;
  alpha = 0.1;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  count += 1;
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

// get an exponential moving average distance
  if (count == 1) {
    dist_ema = 0; }
  else { 
    dist_ema = alpha*dist_raw + (1 - alpha)*dist_ema_prev;
    dist_ema_prev = dist_ema; }

// output the read value to the serial port
  Serial.print("Min:180,raw:"); // 100mm 180mm
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  // 현재 각도
  Serial.println(",Max:360"); // 220mm 300mm

// adjust servo position according to the USS read value

  if(dist_ema < 180.0) {
    myservo.writeMicroseconds(_DUTY_MIN); }
  else if(dist_ema < 360.0){
    int neu = _DUTY_MIN + (dist_ema - _DIST_MIN) * 10;
    myservo.writeMicroseconds(neu); }
  else {
    myservo.writeMicroseconds(_DUTY_MAX); }
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  float reading;
  digitalWrite(TRIG, HIGH);  // 발신
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);   // 발신 종료

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) {
    reading = 0.0;
    analogWrite(PIN_LED, 255); } // return 0 when out of range.
  else {
    analogWrite(PIN_LED, 0); }

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
