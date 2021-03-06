#include "include/honeywell.h"

int led1          = D7;
int ultrasonicPin = A0;

char jsonBuf[2048];

SystemSleepConfiguration config;


int delayLength = 5000;

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

  // 3.243 appears to be nominal voltage

  // V_cc / 1024 per cm
  // Meanwhile the output is a number between 0 and 4095
  // So 4095/1024 = 1 cm
  //   (or ~4 per cm)
  // Therefore, divide by 4 to get the number of cm
  // Then, to get ft, divide by 2.54, then divide by 12
  return val / (4.0 * 2.54 * 12);
}

float captureDistanceReading(int pin, int n_samples=1, int sample_spacing=0) {
  int sum = 0;

  for (int i = 0; i < n_samples; i++) {
    if (sample_spacing > 0 && i > 0)
      delay(sample_spacing);

    sum += analogRead(pin);
  }

  // The conversion from voltage to distance in ft to nearest object is linear,
  // so we can just take the average of all of the voltages in
  // `ultrasonicSampleBuf`.
  float avgVoltage  = (float) sum / n_samples;
  float avgDistance = distanceFromADC(avgVoltage);

  return avgDistance;
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

  // Capture 100 samples spaced 300ms apart
  float distance = captureDistanceReading(ultrasonicPin, 100, 300); // unit: [ft]

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
