#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0 // sound velocity (unit: m/s)
#define INTERVAL 25   // sampling interval (unit: ms)
#define _DIST_MIN 100 // (unit: mm)
#define _DIST_MAX 300 // (unit: mm)

float timeout; // (unit: us)
float dist_min, dist_max, dist_raw; // (unit: mm)
unsigned long last_sampling_time;   // (unit: mm)
float scale; // pulse duration to distance conversion

float dist_1 = 0;
float dist_2 = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_raw = 0.0;
  timeout = (INTERVAL / 2) * 1000.0; // (unit: us)
  scale = 0.001 * 0.5 * SND_VEL;     // (unit: mm/us)
  
  Serial.begin(57600);
  last_sampling_time = 0;
 }
 
 
 void loop() {
  dist_1 = dist_2;
  
  if( millis() < last_sampling_time + INTERVAL ) return;
  
  dist_2 = USS_measure(PIN_TRIG, PIN_ECHO);

  if (dist_2 == 0) {
    dist_raw = dist_1;
  }
  else {
    dist_raw = dist_2;
    dist_1 = dist_2;
  }

  Serial.print("Min:0,");
  Serial.print("Raw:"); Serial.print(dist_raw); Serial.print(",");
  Serial.println("Max:400");
  
  if (dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255); }
  else if (dist_raw == 200.0) {
    analogWrite(PIN_LED, 0); }
  else {
    float bright = abs(200.0 - dist_raw) * 2.55;
    analogWrite(PIN_LED, bright); }
  
  last_sampling_time += INTERVAL;
}


float USS_measure(int TRIG, int ECHO) {
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale;
    
  if ( reading < dist_min || reading > dist_max ) {
    reading = 0.0; }
  return reading; }
