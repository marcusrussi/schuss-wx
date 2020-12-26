int led1 = D0; // Instead of writing D0 over and over again, we'll write led1
// You'll need to wire an LED to this one to see it blink.

int led2 = D7; // Instead of writing D7 over and over again, we'll write led2
// This one is the little blue LED on your board. On the Photon it is next to D7, and on the Core it is next to the USB jack.

char jsonBuf[256];
JSONBufferWriter writer(jsonBuf, sizeof(jsonBuf) - 1);

SystemSleepConfiguration config;

void setup() {

  // We are going to tell our device that D0 and D7 (which we named led1 and
  // led2 respectively) are going to be output (That means that we will be
  // sending voltage to them, rather than monitoring voltage that comes from
  // them)

  // It's important you do this here, inside the setup() function rather than
  // outside it or in the loop function.

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  config.mode(SystemSleepMode::ULTRA_LOW_POWER)
        .duration(2min);

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
  writer.beginObject();
  writer.name("temp").value(12.34);
  writer.name("depth").value(1.01);
  writer.name("swe").value(0.2);
  writer.endObject();
  writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;


  // To blink the LED, first we'll turn it on...
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  Particle.publish("wx", jsonBuf);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);

  delay(30s);

  // System.sleep(config);
  // And repeat!
}
