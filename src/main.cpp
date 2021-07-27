#include <Arduino.h>
#include <Wire.h>
#include <main.h>

#define SCL_PIN 4
#define SCL_PORT PORTB
#define SDA_PIN 5
#define SDA_PORT PORTB

#include <SoftWire.h>
SoftWire Master = SoftWire();
TwoWire Slave = TwoWire();

#define JUNGLE_ADDRESS 0x44

byte rcv = 255;

void setup() {
  delay(2000);
  Serial.begin(115200);
  Master.begin();
  Slave.begin(JUNGLE_ADDRESS);
  Slave.onReceive(slaveReceive);
  Serial.println("Starting...");
  if (!i2c_init()) { 
    Serial.println("Failed to start Softi2cMaster"); 
  } else
  {
    Serial.println("Started");
  }
  
}

void loop() {
  // if(!i2c_start_wait(JUNGLE_ADDRESS << 1|I2C_WRITE)) {
  //   Serial.println("Device is busy");
  //   return;
  // }
  // Serial.println("Master sending");
  // i2c_write(0xA);
  // i2c_write(0xC0);
  // i2c_stop();
  delay(100);
}

void slaveReceive(int n) {
  bool found = false;
  if (Slave.available() < 1) {
    return;
  }
  Serial.println("Slave received");
  if(!i2c_start_wait(JUNGLE_ADDRESS<<1|I2C_WRITE)) {
    Serial.println("Device is busy");
  }
  Serial.println("i2c started");
  Serial.println(Slave.available());
  while(Slave.available()) {
    rcv = Slave.read();
    Serial.println(rcv);
    Serial.println(i2c_write(rcv));
  }
  i2c_stop();
  Serial.println("After stop");
}