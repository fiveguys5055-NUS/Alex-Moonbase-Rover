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

#define ESTOP_BIT   PD3   // INT3
#define TCS_S0_BIT  PA0
#define TCS_S1_BIT  PA1
#define TCS_S2_BIT  PA2
#define TCS_S3_BIT  PA3
#define TCS_OUT_BIT PD2   // INT2
#define ESTOP_BIT PD3
#define DEBOUNCE_MS 50

volatile TState  buttonState   = STATE_RUNNING;
volatile bool    stateChanged  = false;
static volatile uint32_t _lastDebounceTime = 0;
static volatile int count=0;
static volatile int latest_time=0;

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




// =============================================================
// Robotic Arm — 4-DoF MeArm v0.4 on Port K (A8-A11)
// Timer1 CTC, prescaler 8 (2 MHz), 5ms per servo slot x 4 = 20ms
// =============================================================

volatile int arm_tracker = 1;
volatile int arm_reference;
volatile int OCR_B = 2837;
volatile int OCR_S = 2630;
volatile int OCR_E = 2837;
volatile int OCR_G = 2837;
volatile int B_target = 2837;
volatile int S_target = 2630;
volatile int E_target = 2837;
volatile int G_target = 2837;
int msPerDeg = 10;
static unsigned long arm_lastupdate = 0;

static int arm_parse3(const char *s) {
    if (s[0] < '0' || s[0] > '9') return -1;
    if (s[1] < '0' || s[1] > '9') return -1;
    if (s[2] < '0' || s[2] > '9') return -1;
    return (s[0] - '0') * 100 + (s[1] - '0') * 10 + (s[2] - '0');
}

void moveSmooth(volatile int *actual_OCR, int new_OCR, int step) {
    if (*actual_OCR < new_OCR) {
        *actual_OCR += step;
        if (*actual_OCR > new_OCR) *actual_OCR = new_OCR;
    }
    gf (*actual_OCR > new_OCR) {
        *actual_OCR -= step;
        if (*actual_OCR < new_OCR) *actual_OCR = new_OCR;
    }
}

void homeAll() {
    B_target = 2837; S_target = 3251;
    E_target = 2009; G_target = 2837;
}

void setupTimer5() {
    cli();
    TCCR5A = 0;
    TCCR5B = 0b00001010;  // Prescaler 8 (2MHz clock)
    TCNT5 = 0;
    TIMSK5 |= 0b110;
    OCR5A = 9999;  // 5ms per slot
    OCR5B = 2837;
    sei();
}

ISR(TIMER5_COMPA_vect) {
    if (arm_tracker == 4)      { OCR5B = OCR_G; PORTK |= 0b00001000; }
    else if (arm_tracker == 3) { OCR5B = OCR_E; PORTK |= 0b00000100; }
    else if (arm_tracker == 2) { OCR5B = OCR_S; PORTK |= 0b00000010; }
    else                       { OCR5B = OCR_B; PORTK |= 0b00000001; }
}

ISR(TIMER5_COMPB_vect) {
    arm_reference = arm_tracker;
    if (arm_reference == 4)      PORTK &= 0b11110111;
    else if (arm_reference == 3) PORTK &= 0b11111011;
    else if (arm_reference == 2) PORTK &= 0b11111101;
    else                         PORTK &= 0b11111110;
    arm_tracker++;
    if (arm_tracker > 4) arm_tracker = 1;
}

static void handleArmCommand(const char *data) {
    char cmd_char = data[0];
    // Debug: send what we received
    {
        TPacket dbg;
        memset(&dbg, 0, sizeof(dbg));
        dbg.packetType = PACKET_TYPE_MESSAGE;
        snprintf(dbg.data, 32, "ARM:%c%c%c%c T5B=%d", data[0], data[1], data[2], data[3], (int)OCR5B);
        sendFrame(&dbg);
    }
    if (cmd_char == 'H' || cmd_char == 'h') { homeAll(); return; }
    if (strlen(data) < 4) return;
    int val = arm_parse3(&data[1]);
    if (val < 0) return;
    int vel = val;
    if (val > 180) val = 180;
    if (val < 0) val = 0;
    int OCR_val = ((double)val / 180.0) * 3725.0 + 975;
    switch (cmd_char) {
        case 'V': case 'v': msPerDeg = vel; break;
        case 'B': case 'b': B_target = OCR_val; {
            TPacket dbg2; memset(&dbg2, 0, sizeof(dbg2)); dbg2.packetType = PACKET_TYPE_MESSAGE;
            snprintf(dbg2.data, 32, "Bt=%d OCR_B=%d", B_target, (int)OCR_B);
            sendFrame(&dbg2);
        } break;
        case 'S': case 's':
            if (val > 110) val = 110;
            OCR_val = ((double)val / 180.0) * 3725.0 + 975;
            S_target = OCR_val; break;
        case 'E': case 'e':
            if (val > 160) val = 160;
            if (val < 25) val = 25;
            OCR_val = ((double)val / 180.0) * 3725.0 + 975;
            E_target = OCR_val; break;
        case 'G': case 'g':
            if (val < 77) val = 77;
            if (val > 110) val = 110;
            OCR_val = ((double)val / 180.0) * 3725.0 + 975;
            G_target = OCR_val; break;
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
volatile uint8_t motorSpeed = 120;
volatile uint8_t turnSpeed = 145;
volatile TCommandType last_cmd;

#define MOVE_DURATION_MS 400
#define TURN_DURATION_MS 400
static volatile unsigned long movementStartTime = 0;
static volatile bool movementActive = false;
static volatile unsigned long movementDuration = MOVE_DURATION_MS; 

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

        case COMMAND_RUN:
            cli();
            buttonState  = STATE_RUNNING;
            stateChanged = false;
            sei();
            sendOK();
            sendStatus(STATE_RUNNING);
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
            if (buttonState == STATE_RUNNING) { forward(motorSpeed); movementStartTime = millis(); movementActive = true; movementDuration = MOVE_DURATION_MS; }
            sendOK();
            last_cmd = COMMAND_FW;
            break;

        case COMMAND_BW:
            if (buttonState == STATE_RUNNING) { backward(motorSpeed); movementStartTime = millis(); movementActive = true; movementDuration = MOVE_DURATION_MS; }
            sendOK();
            last_cmd = COMMAND_BW;
            break;

        case COMMAND_LEFT:
            if (buttonState == STATE_RUNNING) { ccw(turnSpeed); movementStartTime = millis(); movementActive = true; movementDuration = TURN_DURATION_MS; }
            sendOK();
            last_cmd = COMMAND_LEFT;
            break;

        case COMMAND_RIGHT:
            if (buttonState == STATE_RUNNING) { cw(turnSpeed); movementStartTime = millis(); movementActive = true; movementDuration = TURN_DURATION_MS; }
            sendOK();
            last_cmd = COMMAND_RIGHT;
            break;

        case COMMAND_FAST:
            if (motorSpeed <= 235) motorSpeed += 20;
            if (turnSpeed <=235) turnSpeed += 20;
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
            if (turnSpeed >= 20) turnSpeed -= 20;
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

        case COMMAND_ARM:
            if (buttonState == STATE_RUNNING) {
                handleArmCommand(cmd->data);
            }
            sendOK();
            break;
    }
}

// ------------------------------------------------------------------
// setup / loop
// ------------------------------------------------------------------

void setup() {
    
    //usartInit(103);   // 9600 baud at 16 MHz

    Serial.begin(115200);

    // E-Stop button: PD3 (INT3), input (external pull-down)
    DDRD  &= ~(1 << ESTOP_BIT);   // PD3 as input
    EICRA |=  (1 << ISC30);       // any logical change: ISC31=0, ISC30=1
    EICRA &= ~(1 << ISC31);
    EIMSK |=  (1 << INT3);        // enable INT3
    
    //setup colour sensor    
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

    // Arm servos: Port K bits 0-3 (A8-A11) as outputs + start Timer1
    DDRK |= 0b00001111;
    setupTimer5();

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

    // Timed-pulse auto-stop
    if (movementActive && (millis() - movementStartTime >= movementDuration)) {
        stop();
        movementActive = false;
    }

    // Smooth arm servo motion update
    unsigned long currenttime = millis();
    if (currenttime - arm_lastupdate >= (unsigned long)msPerDeg) {
        arm_lastupdate = currenttime;
        cli();
        moveSmooth(&OCR_B, B_target, 23);
        moveSmooth(&OCR_S, S_target, 23);
        moveSmooth(&OCR_E, E_target, 23);
        moveSmooth(&OCR_G, G_target, 23);
        sei();
    }

    // Process incoming commands from the Pi
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}

