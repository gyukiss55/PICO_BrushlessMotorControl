#include <Arduino.h>

static unsigned long timePeriod;
static unsigned long timeBLState;
static unsigned long timeLED;
static uint16_t dutyValue;
static uint8_t stateBL;
static bool startBLChange;

static uint32_t counterDuty = 0;
static unsigned long counterBLState = 0;

static bool ledOn = false;

#define BLO1_PIN 10
#define BLO2_PIN 11
#define BLO3_PIN 12
#define PWM_PIN 13  // Use GPIO12 for PWM output

#define BUILTIN_LED 0
#define PWM_RANGE (4096 - 1)
#define PWM_RESOLUTION 12
#define PWM_FREQ 1000
// 1 kHz = 1000 msec
#define DUTY_PERIOD 10L * 1000L * 1000L / (PWM_RANGE + 1)
// 10L * 1000L * 1000L = 10 sec
#define BLSTATE_PERIOD 10L * 1000L
// 10L * 1000L = 10 msec

void setup() {
   Serial.begin(115200);
   pinMode(BLO3_PIN, OUTPUT);
   init_values(); 
}


void loop() {
  loop_duty ();
  loop_BLState ();
}

void init_values ()
{
  // Configure PWM_PIN as output
  pinMode(PWM_PIN, OUTPUT);
  pinMode(BLO1_PIN, OUTPUT);
  pinMode(BLO2_PIN, OUTPUT);
  pinMode(BLO3_PIN, OUTPUT);

  digitalWrite (BLO1_PIN, 0);
  digitalWrite (BLO2_PIN, 0);
  digitalWrite (BLO3_PIN, 0);

  // Set the PWM frequency to 100 Hz (i.e., 10 ms period)
  // This function may be available in your Arduino core for RP2040.
  analogWriteRange (PWM_RANGE);
  analogWriteFreq(PWM_FREQ);
  // (Optional) Ensure we use 8-bit resolution (0-255)
  // The default resolution is typically 8 bits, but you can set it explicitly if needed.
  analogWriteResolution(PWM_RESOLUTION);  

  timePeriod = time_us_64(); // Get the initial timestamp 
  timeBLState = time_us_64(); // Get the initial timestamp 
  dutyValue = 0;
  stateBL = 0;
  startBLChange = true;
  analogWrite(PWM_PIN, dutyValue); 

}

void writeBLStatePort (byte state)
{
  digitalWrite (BLO1_PIN, state & 1 ? HIGH : LOW);
  digitalWrite (BLO2_PIN, state & 2 ? HIGH : LOW);
  digitalWrite (BLO3_PIN, state & 4 ? HIGH : LOW);

}

void loop_duty() {
  unsigned long ts = time_us_64();
  if (ts  - timePeriod > DUTY_PERIOD) {
    if (dutyValue == 0) {
      Serial.print (ts); 
      Serial.print (","); 
      Serial.print (timePeriod); 
      Serial.print (","); 
      Serial.print (ts  - timePeriod); 
      Serial.print (",");

      Serial.print (counterDuty++); 
      Serial.print (". duty cycle ");
      Serial.print (counterBLState); 
      Serial.println (". BL cycle");
      digitalWrite (BUILTIN_LED, LOW);
      ledOn = true;
      timeLED = ts;

    }
    timePeriod = ts;
    if (dutyValue < PWM_RANGE)
      dutyValue++;
    else 
      dutyValue = 0;
    analogWrite(PWM_PIN, dutyValue);
  }
  if (ledOn && ((ts - timeLED) > (1000 * 1000))){
    ledOn = false;
    digitalWrite (BUILTIN_LED, HIGH);
    Serial.print (ts); 
    Serial.println (" LED OFF");

  }

}

void loop_BLState() {
  unsigned long ts = time_us_64();
  if (ts  - timeBLState > BLSTATE_PERIOD) {
    timeBLState = ts;
    stateBL++;
    if (stateBL >= 0b111)
      stateBL == 0b001;
    startBLChange = true;
    writeBLStatePort(0);
    counterBLState++; 
  } else if (startBLChange) {
    startBLChange = false;
    writeBLStatePort(stateBL); 
  }

}

void simple_loop() {
  // Ramp the PWM duty cycle from 0 to 255 over 10 seconds.
  // With 256 steps, each step should last ~39 ms (10,000 ms / 256 ≈ 39 ms)
  for (int duty = 0; duty <= 255; duty++) {
    analogWrite(PWM_PIN, duty);   // Update duty cycle
    delay(10000 / PWM_RANGE);           // Wait ~39 ms between steps
  }
  
  // Optionally, you could reverse the ramp or restart from 0.
  // Here, we simply restart the ramp from 0.
}
