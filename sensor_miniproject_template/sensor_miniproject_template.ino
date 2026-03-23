/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// Pin assignments
// =============================================================
//
// E-Stop button : Pin 18 (PD3, INT3)
//   Wiring: button between 5 V and Pin 18, 10 k pull-down to GND.
//   Pin reads HIGH when pressed, LOW when released.
//
// TCS3200 Color Sensor:
//   S0  -> Pin 22 (PA0)   S1  -> Pin 23 (PA1)
//   S2  -> Pin 24 (PA2)   S3  -> Pin 25 (PA3)
//   OUT -> Pin 19 (PD2, INT2)
//   VCC -> 5 V            GND -> GND

#define ESTOP_BIT   PD3   // INT3
#define TCS_S0_BIT  PA0
#define TCS_S1_BIT  PA1
#define TCS_S2_BIT  PA2
#define TCS_S3_BIT  PA3
#define TCS_OUT_BIT PD2   // INT2

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

static volatile uint32_t _lastDebounceTime = 0;
#define DEBOUNCE_MS 50
static volatile int count=0;
static volatile int latest_time=0;
/*ISR(INT3_vect) {
  // YOUR CODE HERE
  unsigned long curr_time = millis();
  
  if(curr_time - latest_time > 50) {
  latest_time = curr_time;

  bool isHigh = (PIND & (1 << PIND2));

  if(isHigh) { //RISING EDGE, count 0 or count 2
    buttonState = STATE_STOPPED;
    if (count == 0) count = 1; else if (count == 2) count = 3;
  }
  else { //FALLING EDGE, count 1 or count 3 
    if(count == 1) {
      buttonState = STATE_STOPPED; count = 2; 
    }
    else if(count == 3) {
      buttonState = STATE_RUNNING; count = 0;
    }
  }
  stateChanged = true;
}
}
*/
// Fires on any logical change of the E-Stop button pin (INT3).
ISR(INT3_vect) {
    uint32_t now = millis();
    if (now - _lastDebounceTime < DEBOUNCE_MS) return;
    _lastDebounceTime = now;
    count++;
    if(count==1 || count==4){
    uint8_t pin = (PIND >> ESTOP_BIT) & 1;
    if (buttonState == STATE_RUNNING && pin == 1) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
    } else if (buttonState == STATE_STOPPED && pin == 0) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
    }
    if(count==4)
    count=0;
    }


// =============================================================
// Color sensor (TCS3200)
// =============================================================

// Rising-edge counter for the TCS3200 OUT pin (INT2).
static volatile uint32_t _colorEdgeCount = 0;

// ISR: increment edge counter on each rising edge of TCS3200 OUT.
ISR(INT2_vect) {
    _colorEdgeCount++;
}

/*
 * Measure one color channel over a 100 ms window by counting rising
 * edges on the TCS3200 OUT pin via INT2.
 *
 * s2, s3: 0 or 1 to select the channel (see TCS3200 datasheet).
 * Returns the edge count (NOT yet multiplied by 10).
 */
static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    // Set S2 / S3 channel select pins
    if (s2) PORTA |= (1 << TCS_S2_BIT); else PORTA &= ~(1 << TCS_S2_BIT);
    if (s3) PORTA |= (1 << TCS_S3_BIT); else PORTA &= ~(1 << TCS_S3_BIT);

    // Allow the sensor output frequency to settle (~10 ms)
    uint32_t settle = millis();
    while (millis() - settle < 10) {}

    // Reset counter, then enable INT2 to start counting rising edges
    cli();
    _colorEdgeCount = 0;
    sei();
    EIMSK |= (1 << INT2);

    // Count for exactly 100 ms
    uint32_t start = millis();
    while (millis() - start < 100) {}

    // Stop counting and read result atomically
    EIMSK &= ~(1 << INT2);
    cli();
    uint32_t count = _colorEdgeCount;
    sei();

    return count;
}

/*
 * Measure all three color channels and convert to Hz.
 * Hz = edge_count * 10  (for a 100 ms window).
 *
 * S2/S3 channel selection (from TCS3200 datasheet):
 *   Red:   S2=L, S3=L  -> measureChannel(0, 0)
 *   Green: S2=H, S3=H  -> measureChannel(1, 1)
 *   Blue:  S2=L, S3=H  -> measureChannel(0, 1)
 */
static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannel(0, 0) * 10;
    *g = measureChannel(1, 1) * 10;
    *b = measureChannel(0, 1) * 10;
}

static void sendOK()              { sendResponse(RESP_OK,     0); }
static void sendStatus(TState st) { sendResponse(RESP_STATUS, (uint32_t)st); }

// ------------------------------------------------------------------
// Motor speed (0-255).  Start at 150; FAST/SLOW adjust by 25.
// ------------------------------------------------------------------
volatile uint8_t motorSpeed = 100;
volatile TCommandType last_cmd;


// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {

        case COMMAND_ESTOP:
            stop();
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            sendOK();
            sendStatus(STATE_STOPPED);
            break;

        case COMMAND_COLOR:
            uint32_t r, g, b;
            readColorChannels(&r, &g, &b);
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            pkt.params[0]  = r;
            pkt.params[1]  = g;
            pkt.params[2]  = b;
            sendFrame(&pkt);
            break;


        case COMMAND_FW:
            if (buttonState == STATE_RUNNING) forward(motorSpeed);
            sendOK();
            last_cmd = COMMAND_FW;
            break;

        case COMMAND_BW:
            if (buttonState == STATE_RUNNING) backward(motorSpeed);
            sendOK();
            last_cmd = COMMAND_BW;
            break;

        case COMMAND_LEFT:
            if (buttonState == STATE_RUNNING) ccw(motorSpeed);
            sendOK();
            last_cmd = COMMAND_LEFT;
            break;

        case COMMAND_RIGHT:
            if (buttonState == STATE_RUNNING) cw(motorSpeed);
            sendOK();
            last_cmd = COMMAND_RIGHT;
            break;

        case COMMAND_FAST:
            if (motorSpeed <= 235) motorSpeed += 20;
            if(last_cmd == COMMAND_FW) {
              forward(motorSpeed);
            }
            else if(last_cmd == COMMAND_BW) {
              backward(motorSpeed);
            }
            else if(last_cmd == COMMAND_LEFT) {
              ccw(motorSpeed);
            }
            else if(last_cmd == COMMAND_RIGHT) {
              cw(motorSpeed);
            }

            sendOK();
            break;

        case COMMAND_SLOW:
            if (motorSpeed >= 20) motorSpeed -= 20;
            if(last_cmd == COMMAND_FW) {
              forward(motorSpeed);
            }
            else if(last_cmd == COMMAND_BW) {
                backward(motorSpeed);
            }
            else if(last_cmd == COMMAND_LEFT) {
              ccw(motorSpeed);
            }
            else if(last_cmd == COMMAND_RIGHT) {
              cw(motorSpeed);
            }

            sendOK();
            break;
    }
            
}
// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // --- E-Stop button: Pin 18 = PD3 = INT3, input (external 10 k pull-down) ---
    DDRD  &= ~(1 << ESTOP_BIT);          // PD3 as input
    // INT3: any logical change -> ISC31=0, ISC30=1
    EICRA |=  (1 << ISC30);
    EICRA &= ~(1 << ISC31);
    EIMSK |=  (1 << INT3);              // enable INT3

    // --- TCS3200 control pins: S0-S3 on Port A, outputs ---
    DDRA  |= (1 << TCS_S0_BIT) | (1 << TCS_S1_BIT)
           | (1 << TCS_S2_BIT) | (1 << TCS_S3_BIT);
    // 20 % frequency scaling: S0=HIGH, S1=LOW
    PORTA |=  (1 << TCS_S0_BIT);
    PORTA &= ~(1 << TCS_S1_BIT);

    // --- TCS3200 OUT: Pin 19 = PD2 = INT2, input ---
    DDRD  &= ~(1 << TCS_OUT_BIT);       // PD2 as input
    // INT2: rising edge -> ISC21=1, ISC20=1 (INT2 NOT enabled here; enabled per measurement)
    EICRA |= (1 << ISC21) | (1 << ISC20);

    sei();
}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
