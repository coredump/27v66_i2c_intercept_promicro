/* This is HEAVILY based on the JVC mod made by Martin Hejnfelt 
   and available here https://github.com/skumlos/tb1226en-i2c-bridge
   Altered to be used on a Pro Micro and Platform.io stuff
*/

#include <Arduino.h>
#include <Wire.h>
#include <RingBufCPP.h>
#include <singleLEDLibrary.h>

#define DELAY_MS 100

#define SCL_PIN 4 // PB4 - Pin 8 on a Pro Micro
#define SCL_PORT PORTB
#define SDA_PIN 5 // PB5 - Pin 9 on a Pro Micro
#define SDA_PORT PORTB
#define I2C_PULLUP 0
#define I2C_FASTMODE 0

#include <SoftI2CMaster.h>

// Address in the datasheet is said to be 0x88 for write
// and 0x89 for read. That is somewhat of a "mistake" as
// i2c uses 7 bit addressing and the least significant bit
// is read (1) or write (0). Thus the address is shifted once
// to the right to get the "real" address which is then 44h/68
#define JUNGLE_ADDR (0x44)

/* For the CXA2133S this seems to work
   There are no datasheets for it, but the CXA2061S seems to be 
   very close.
*/
#define REG_RGB (0xA)
#define REG_INIT (0x9)

/* Easier than writing 11111110 all the time */
#define MASK (0xfe)

/* Theoretically this will be enough (famous last words) */
#define BUFFER_SIZE (32)

void writeRequest(int);
void readRequest();

RingBufCPP<int, BUFFER_SIZE> buf;
sllib led(LED_BUILTIN_RX);

uint8_t r[2] = {0, 0};
unsigned long start = millis();
bool iicinit = false;

void setup()
{
  Serial.begin(115200);
  Serial.print("Sony Jungle I2C Bridge\n");
  led.setBlinkSingle(1000);

  iicinit = i2c_init();
  Wire.setClock(100000);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  Wire.begin(JUNGLE_ADDR);
  pinMode(LED_BUILTIN_RX, OUTPUT);

  if (!iicinit)
  {
    Serial.println("I2C init failed");
    int pattern[] = {1000, 200, 500, 200, 500, 200};
    led.setPatternSingle(pattern, 6);
  }
}

// void writeRegister(const uint8_t reg, const uint8_t val)
// {
//   i2c_start((JUNGLE_ADDR << 1) | I2C_WRITE);
//   i2c_write(reg);
//   i2c_write(val);
//   i2c_stop();
// }

void readRequest()
{
  Wire.write(r, 2);
}

void writeRequest(int byteCount)
{
  bool reg_found = false;
  bool single_reg_found = false;
  int c = 0;
  int data = 0;

  while (Wire.available())
  {
    data = Wire.read();
    if (byteCount > 2)
    {
      if (data == REG_INIT)
      {
        reg_found = true;
      }

      if (reg_found && (c == 2))
      {
        data = data & MASK;
      }
      c++;
    }
    else if (byteCount == 2)
    {
      if (data == REG_RGB)
      {
        single_reg_found = true;
      }

      if (single_reg_found && Wire.available() == 0)
      {
        data = data & MASK;
      }
    }
    buf.add(data);
  }
}

void loop()
{
  int d;
  if (buf.numElements() > 0)
  {
    if(!i2c_start(JUNGLE_ADDR << 1 | I2C_WRITE)) {
      led.setBlinkSingle(100);
    }
    else {
      led.setBlinkSingle(1000);
    }
    while (buf.pull(&d))
    {
      i2c_write(d);
    }
    i2c_stop();
  }
  led.update();
}