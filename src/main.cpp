#include <Arduino.h>
#include <Wire.h>
#include <singleLEDLibrary.h>

/* We use this circular buffer to be able to have really fast i2c slave
   handling functions (since they are actually interrupt handlers). That
   way we can receive all data and add to the buffer and then deal with it
   when the MCU isn't handling some other i2c data
*/
#define CIRCULAR_BUFFER_INT_SAFE
#include <CircularBuffer.h>

#define SDA_PORT PORTB
#define SDA_PIN 5 // Pin 9 on pro micro
#define SCL_PORT PORTB
#define SCL_PIN 4 // Pin 8 on pro micro
#define I2C_PULLUP 0
#define I2C_FASTMODE 0
#include <SoftI2CMaster.h>

#define JUNGLE_ADDR (0x44)

sllib led1(LED_BUILTIN_RX);
CircularBuffer<byte, 64> sBuf;

/* This is a heavy handed solution for the problem with Micom reads.
   Every few seconds the Micom reads the status from the Jungle chip
   to make sure the TV isn't on fire or something. If those tests fail
   it shuts off the cannon and turns off the TV. So instead of reading
   this status from the jungle and sending (that takes forever) we will
   just send the status and hope the TV isn't actually on fire.
*/
volatile byte mRCount = 0;
byte dataInit[] = {0x00, 0x85};
byte dataOK[] = {0x40, 0x85};

void slaveReceive(int n);
void slaveRequest();

void setup() {
  Wire.begin(JUNGLE_ADDR);
  Wire.onReceive(slaveReceive);
  Wire.onRequest(slaveRequest);

  /* We don't want the Pullups to be enabled. This seem to be the way to do it.
   */
  // digitalWrite(2, LOW);
  // digitalWrite(3, LOW);

  if (!i2c_init()) {
    led1.setBlinkSingle(200);
  }

  // Put a heartbeat led to make things easier to see
  led1.setBlinkSingle(1000);
}

/* Those two functions are ISRs so be fast and follow all
   ISR tips and tricks
*/
void slaveReceive(int n) {
  while (Wire.available()) {
    byte c = Wire.read();
    sBuf.push(c);
  }
}

/* During my signal analysis of the i2c protocol that this TV uses, I verified
   that usually takes around 6 requests for the IKR bit to stabilize and flip
   (what finishes the startup self check). So we literally count how many times
   we got asked for the register and then send the OK looking data back.
   Ugly but it works.
*/
void slaveRequest() {
  if (mRCount > 7) {
    Wire.write(dataOK, 2);
  } else {
    Wire.write(dataInit, 2);
    mRCount++;
  }
}

void loop() {
  byte bSize = sBuf.size(); // if there's data from the micom, this will be > 0
  if (bSize > 0) {
    byte data[bSize];
    byte i = 0;

    while (!sBuf.isEmpty()) {
      byte d = sBuf.shift();
      data[i] = d;
      ++i;
    }

    /* The Micom writes to the Jungle in groups of three bytes/registers at a
       time, so the register that we want is actually on 0xA that is included
       on the group starting on 0x9. We use a & to mask and change only the
       last bit to 0 and enable the RGB SEL bit.
       Also, theoretically using a SWITCH with ascending int cases makes
       the MCU spend less cycles, so why the heck not.
    */
    switch (data[0]) {
    case 0x9:
      data[2] = (data[2] & 0xfe);
      break;

    /* In didn't see this being called directly, but just in case it happens
       sometime, we also mask the RGB SEL bit if we receive any data for the 0XA
       register
    */
    case 0xA:
      data[1] = (data[1] & 0xfe);
      break;

    default:
      break;
    }

    if (!i2c_start((JUNGLE_ADDR << 1) | I2C_WRITE)) {
      led1.setBlinkSingle(500);
      return;
    }
    for (byte j = 0; j < bSize; j++) {
      i2c_write(data[j]);
    }
    i2c_stop();
  }

  led1.update();
}