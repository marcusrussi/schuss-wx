#include "include/honeywell.h"

#define US_NUM_SAMPLES 100  // Number of samples to collect on each loop
#define US_MIN_DIST    15   // unit: [in]
#define US_MAX_DIST    57   // unit: [in]

#define JSON_BUFLEN    8192

// Pin mappings for the signaling LED, and the ultrasonic ADC pin
int led1          = D7;
int ultrasonicPin = A0;

// Buffers for constructing JSON objects, and for collecting samples
// off the ultrasonic sensor
char jsonBuf[JSON_BUFLEN];

SystemSleepConfiguration config;

int delayLength = 10000;

TruStabilityPressureSensor pressureSensor(
  D14, // SS pin for SPI
  0,   // Min pressure in PSI
  1,   // Max pressure for PSI
  SPISettings(100000, MSBFIRST, SPI_MODE0) // 100kHz freq
);

float CtoF (float C) {
  return C * (9/5) + 32;
} 

float distanceFromADC(float val) {

  // 3.243 appears to be nominal voltage, however this doesn't factor into the
  // calculation because ADC output is as a fraction of Vcc, not of the 
  // "stated" 3.3V

  // V_cc / 1024 per cm
  // V_cc / 1024 per 5 mm
  // Meanwhile the output is a number between 0 and 4095
  // So 4095/1024 = 1 cm
  // So 4095/1024 = 1 cm
  //   (or ~4 per cm)
  // Therefore, divide by 4 to get the number of cm
  // Then, to get ft, divide by 2.54, then divide by 12
  return (1/2.54) * (1/10.0)  * val * (5120.0 / 4095.0);
}

float captureDistanceReading(int pin,
                             int sample_spacing,
                             float min_dist,
                             float max_dist)
{
  float max_dist_seen    = 0;
  int   successful_reads = 0;

  int   sensorReadRaw;
  float sensorReadIn;

  for (int i = 0; i < US_NUM_SAMPLES; i++) {
    if (sample_spacing > 0 && i > 0)
      delay(sample_spacing);

    sensorReadRaw = analogRead(pin);
    sensorReadIn  = distanceFromADC(sensorReadRaw);

    if (min_dist < sensorReadIn && sensorReadIn <= max_dist) {
      ++successful_reads;
      
      if (sensorReadIn > max_dist_seen)
        max_dist_seen = sensorReadIn;
    }
  }

  if (successful_reads == 0)
    return 0;

  return max_dist_seen;
}

void setup() {

  pinMode(led1, OUTPUT);

  // For debugging
  Serial.begin(9600);
  waitFor(Serial.isConnected, 30000);
  Serial.println("[penguin] Connected to serial");

  config.mode(SystemSleepMode::ULTRA_LOW_POWER).duration(2min);

  SPI.begin();
  pressureSensor.begin();
}

// Next we have the loop function, the other essential part of a
// microcontroller program.
//
// This routine gets repeated over and over, as quickly as possible and as many
// times as possible, after the setup function is called.
//
// Note: Code that blocks for too long (like more than 5 seconds), can make
// weird things happen (like dropping the network connection).  The built-in
// delay function shown below safely interleaves required background activity,
// so arbitrarily long delays can safely be done if you need them.

void loop() {

  int time       = Time.now();                            // unit: [s]
  float temp     = CtoF(pressureSensor.temperature());    // unit: [F]
  float pressure = pressureSensor.pressure();             // unit: [psi]

  // Capture 1000 samples spaced 30ms apart
  // float distance = captureDistanceReading(
  //     ultrasonicPin,  // Which ADC pin
  //     30,             // Every 30ms
  //     US_MIN_DIST,    // Min distance for windowed average
  //     US_MAX_DIST     // Max distance for windowed average
  // ); // unit: [ft]

  float distance = captureDistanceReading(
      ultrasonicPin,  // Which ADC pin
      600,            // Every 600ms
      US_MIN_DIST,    // Min distance for windowed average
      US_MAX_DIST     // Max distance for windowed average
  ); // unit: [in]

  JSONBufferWriter writer(jsonBuf, sizeof(jsonBuf));
  memset(jsonBuf, 0, sizeof(jsonBuf));

  writer.beginObject();
    writer.name("time")
      .value( time );
    writer.name("temp")
      .value( temp );
    writer.name("pressure")
      .value( pressure );
    writer.name("distance")
      .value( distance );
    writer.name("swe")
      .nullValue();
  writer.endObject();

  if( pressureSensor.readSensor() == 0 ) {
    Serial.print( "temp [C]: " );
    Serial.print( temp );
    Serial.print( "\t pressure [psi]: " );
    Serial.print( pressure );
    Serial.print( "\t distance [ft]: " );
    Serial.println( distance );
  }

  digitalWrite(led1, HIGH);
    Particle.publish("wx", jsonBuf);
    delay(100);
  digitalWrite(led1, LOW);

  delay(delayLength);

  // System.sleep(config);
}
