/* This is HEAVILY based on the JVC mod made by Martin Hejnfelt 
   and available here https://github.com/skumlos/tb1226en-i2c-bridge
   Altered to be used on a Pro Micro and Platform.io stuff
*/

#include <Arduino.h>
#include <Wire.h>

#define DELAY_MS 100

#define SCL_PIN 4 // PB4 - Pin 8 on a Pro Micro
#define SCL_PORT PORTB
#define SDA_PIN 5 // PB5 - Pin 9 on a Pro Micro
#define SDA_PORT PORTB
#define I2C_PULLUP 0
#define I2C_FASTMODE 1

#include <SoftI2CMaster.h>
#include <Wire.h>

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

void writeRequest(int);
void readRequest();

uint8_t r[2] = {0, 0};
unsigned long start = millis();
bool iicinit = false;
bool masked = false;

void setup()
{
  Serial.begin(115200);
  Serial.print("Sony Jungle I2C Bridge\n");
  iicinit = i2c_init();
  Wire.begin(JUNGLE_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!iicinit)
  {
    Serial.println("I2C init failed");
  }
}

void writeRegister(const uint8_t reg, const uint8_t val)
{
  i2c_start((JUNGLE_ADDR << 1) | I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop();
}

void writeRequest(int byteCount)
{
  if (byteCount > 2)
  {
    uint8_t reg = Wire.read();
    uint8_t bc = 1;
    i2c_start((JUNGLE_ADDR << 1) | I2C_WRITE);
    i2c_write(reg);
    do
    {
      uint8_t val = Wire.read();
      switch (reg)
      {
      case REG_RGB:
        // mask the value to make the last bit 0 without changing anything else
        val = (val & 0b11111110);
        masked = true;
        break;
      default:
        break;
      }
      i2c_write(val);
      ++bc;
      ++reg;
    } while (bc < byteCount);
    i2c_stop();
  }
  else
  {
    uint8_t reg = Wire.read();
    uint8_t val = Wire.read();
    switch (reg)
    {
    case REG_RGB:
      writeRegister(reg, (val & 0b11111110));
      masked = true;
      break;
    default:
      writeRegister(reg, val);
      break;
    }
  }
}

void readRequest()
{
  Wire.write(r, 2);
}

void read()
{
  i2c_start((JUNGLE_ADDR << 1) | I2C_READ);
  r[0] = i2c_read(false); // read one byte
  r[1] = i2c_read(true);  // read one byte and send NAK to terminate
  i2c_stop();             // send stop condition
}

void loop()
{
  noInterrupts();
  read();
  interrupts();
  delay(DELAY_MS);
  if ((millis() - start) >= 1000)
  {
    Serial.println("Alive");
    if (!iicinit)
    {
      Serial.println("IICinit failed!");
    }
    else
    {
      Serial.println("IICinit succeeded");
    }

    if (masked)
    {
      Serial.println("Masked is true");
      masked = false;
    } else
    {
      Serial.println("masked is false");
    }
    
    start = millis();
  }
}