/*
 * Alex.ino  —  Remote-controlled 4WD robot via TPacket over serial.
 *
 * Files in this sketch:
 *   Alex.ino        (this file)  — setup, loop, command handler
 *   robotlib.ino                 — motor wiring and move/forward/backward/ccw/cw/stop
 *   packets.h                    — TPacket protocol (must match pi_sensor_new.py)
 *   serial_driver.h              — sendFrame / receiveFrame framing
 *
 * Pi controls the robot by sending TPacket COMMAND frames:
 *   w / COMMAND_FW    -> forward
 *   s / COMMAND_BW    -> backward
 *   a / COMMAND_LEFT  -> turn left  (CCW)
 *   d / COMMAND_RIGHT -> turn right (CW)
 *   + / COMMAND_FAST  -> increase speed by 25 (max 255)
 *   - / COMMAND_SLOW  -> decrease speed by 25 (min 50)
 *   e / COMMAND_ESTOP -> stop motors, set STOPPED state
 *
 * The robot moves continuously once commanded; the next command changes
 * direction or stops it.  Pressing e (E-Stop) also stops the physical
 * button on PD3 / INT3.
 */

#include "packets.h"
#include "serial_driver.h"
#include <string.h>
#include <avr/interrupt.h>

// ------------------------------------------------------------------
// E-Stop button: Pin 18 = PD3 = INT3
// Wiring: button between 5V and Pin 18; 10k pull-down to GND.
//   Pin reads HIGH when pressed, LOW when released.
// ------------------------------------------------------------------
#define ESTOP_BIT PD3
#define DEBOUNCE_MS 50

volatile TState  buttonState   = STATE_RUNNING;
volatile bool    stateChanged  = false;
static volatile uint32_t _lastDebounce = 0;

ISR(INT3_vect) {
    uint32_t now = millis();
    if (now - _lastDebounce < DEBOUNCE_MS) return;
    _lastDebounce = now;

    uint8_t pin = (PIND >> ESTOP_BIT) & 1;
    if (pin == 1 && buttonState == STATE_RUNNING) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
    } else if (pin == 0 && buttonState == STATE_STOPPED) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
    }
}

// ------------------------------------------------------------------
// Packet helpers
// ------------------------------------------------------------------

static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

static void sendOK()              { sendResponse(RESP_OK,     0); }
static void sendStatus(TState st) { sendResponse(RESP_STATUS, (uint32_t)st); }

// ------------------------------------------------------------------
// Motor speed (0-255).  Start at 150; FAST/SLOW adjust by 25.
// ------------------------------------------------------------------
volatile uint8_t motorSpeed = 205;
volatile TCommandType last_cmd; 

// ------------------------------------------------------------------
// Command handler
// ------------------------------------------------------------------

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
            if (buttonState == STATE_RUNNING) ccw(230);
            sendOK();
            last_cmd = COMMAND_LEFT;
            break;

        case COMMAND_RIGHT:
            if (buttonState == STATE_RUNNING) cw(230);
            sendOK();
            last_cmd = COMMAND_RIGHT;
            break;

        case COMMAND_FAST:
            if (motorSpeed <= 230) motorSpeed += 25;
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
            if (motorSpeed >= 25) motorSpeed -= 25;
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

// ------------------------------------------------------------------
// setup / loop
// ------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    // E-Stop button: PD3 (INT3), input (external pull-down)
    DDRD  &= ~(1 << ESTOP_BIT);   // PD3 as input
    EICRA |=  (1 << ISC30);       // any logical change: ISC31=0, ISC30=1
    EICRA &= ~(1 << ISC31);
    EIMSK |=  (1 << INT3);        // enable INT3

    sei();
}

void loop() {
    // Report physical E-Stop button state changes to the Pi
    if (stateChanged) {
        cli();
        TState s = buttonState;
        stateChanged = false;
        sei();
        if (s == STATE_STOPPED) stop();
        sendStatus(s);
    }

    // Process incoming commands from the Pi
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}

