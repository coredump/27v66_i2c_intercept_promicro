#include <Arduino.h>
#include <RingBufCPP.h>
#include <singleLEDLibrary.h>

#define DELAY_MS 100

#define SLAVE_SCL_PORT PORTD
#define SLAVE_SCL_PIN 1 // PD1 - Pin 2 on a Pro Micro
#define SLAVE_SDA_PORT PORTD
#define SLAVE_SDA_PIN 0 // PD0 - Pin 3 on a Pro Micro
#define SCL_PIN 4       // PB4 - Pin 8 on a Pro Micro
#define SCL_PORT PORTB
#define SDA_PIN 5 // PB5 - Pin 9 on a Pro Micro
#define SDA_PORT PORTB

/* Port manipulation/setup stuff
 */
#define DA_CL_LOW 0b00  // Both low, unsure this is really needed
#define DA_HIGH 0b01    // Only SDA is high
#define CL_HIGH 0b10    // Only SCl is high
#define DA_CL_HIGH 0b11 // Both are High

/* Address in the datasheet is said to be 0x88 for write
 and 0x89 for read. That is somewhat of a "mistake" as
 i2c uses 7 bit addressing and the least significant bit
 is read (1) or write (0). Thus the address is shifted once
 to the right to get the "real" address which is then 44h/68
*/
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
#define BUFFER_SIZE (64)

#define D_EVENT 0
#define S_EVENT 1
#define P_EVENT 2
#define SR_EVENT 3

typedef struct Event {
  byte type;
  byte data;
} Event;

volatile byte state = 0;

RingBufCPP<Event, BUFFER_SIZE> buf;
sllib led(LED_BUILTIN_RX);

int failPtrn1[] = {1000, 200, 500, 200};
int failPtrn2[] = {1000, 200, 500, 200, 500, 200};
int failPtrn3[] = {1000, 200, 500, 200, 500, 200, 500, 200};

long unsigned int start = millis();

void handleCL();
void handleDA();

void setup() {
  Serial.begin(115200);
  Serial.print("Sony Jungle I2C Bridge\n");
  led.setBlinkSingle(1000);

  DDRD = DDRD | B11111100;
  DDRB = DDRB | B11111111;
  attachInterrupt(digitalPinToInterrupt(3), handleCL, RISING);
  attachInterrupt(digitalPinToInterrupt(2), handleDA, CHANGE);
}

void loop() {
  if (buf.numElements() > 0) {
    Serial.println(buf.numElements());
    Event e;
    while (buf.pull(&e)) {
      switch (e.type) {
      case S_EVENT:
        Serial.println("Start event");
        break;

      case P_EVENT:
        Serial.println("Stop event");
        break;

      case D_EVENT:
        Serial.println("Data event");
        break;

      default:
        Serial.println("No event");
        break;
      }
      Serial.print("Data: ");
      Serial.println(e.data);
    }
  }
  led.update();
}

void handleCL() {
  byte portState = (PIND & B00000011);
  Event e;
  switch (portState) {
  case DA_CL_HIGH:
    e = {.type = D_EVENT, .data = 1};
    buf.add(e);
    return;
    break;
  case CL_HIGH:
    e = {.type = D_EVENT, .data = 0};
    buf.add(e);
    return;
    break;

  default:
    break;
  }
}

void handleDA() {
  byte portState = (PIND & B00000011);
  Event e;
  switch (portState) {
  case CL_HIGH:
    e = {.type = S_EVENT, .data = 0};
    buf.add(e);
    return;
    break;
  case DA_CL_HIGH:
    e = {.type = P_EVENT, .data = 0};
    buf.add(e);
    return;
    break;

  default:
    break;
  }
}