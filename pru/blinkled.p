.origin 0
.entrypoint SETUP

#define GPIO2 0x481ac000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT   0x194
#define GPIO_DATAOUT      0x138

#define LED_RED     (1<<2)
#define PAUSE_BTN   (1<<5)

#define INS_PER_US 200
#define INS_PER_DELAY_LOOP 2
#define ON_TIME  (200*1000*(INS_PER_US / INS_PER_DELAY_LOOP))
#define OFF_TIME (200*1000*(INS_PER_US / INS_PER_DELAY_LOOP))

#define PRU0_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

SETUP:
    // Enable the OCP master port
    LBCO r0, C4, 4, 4
    CLR  r0, r0, 4
    SBCO r0, C4, 4, 4
    
TURN_LED_ON:
    MOV r1, GPIO2 | GPIO_SETDATAOUT // for setting gpio bit
    MOV r2, LED_RED
    SBBO r2, r1, 0, 4 // sets led on
    MOV r0, ON_TIME

LED_ON:
    SUB r0, r0, 1
    QBNE LED_ON, r0, 0

TURN_LED_OFF:
    MOV r1, GPIO2 | GPIO_SETDATAOUT // for clearing gpio bit
    MOV r2, LED_RED
    SBBO r2, r1, 0, 4 // sets led off
    MOV r0, OFF_TIME

LED_OFF:
    SUB r0, r0, 1
    QBNE LED_OFF, r0, 0

CHECK_BUTTON:
    MOV r5, GPIO2 | GPIO_DATAOUT //for reading gpio bit
    LBBO r6, r5, 0, 4
    QBBC TURN_LED_ON, r6.t5
    
END:
    MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_0
    HALT


