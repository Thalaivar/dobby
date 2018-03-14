.origin 0 
.entrypoint SETUP

#define GPIO2 0x481ac000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194
#define GPIO_DATAIN 0x138

#define LED_RED (1<<2)
#define DELAY_TIME 10000000

#define PRU1_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

.macro ledon
     MOV r1, GPIO2 | GPIO_SETDATAOUT
     MOV r2, LED_RED
     SBBO r2, r1, 0, 4
.endm

.macro ledoff
     MOV r1, GPIO2 | GPIO_CLEARDATAOUT
     MOV r2, LED_RED
     SBBO r2, r1, 0, 4
.endm

.macro readbutton
     MOV r1, GPIO2 | GPIO_DATAIN //for reading gpio bit
     LBBO r2, r1, 0, 4
.endm

SETUP:
        // Enable the OCP master port
        LBCO &r0, C4, 4, 4
        CLR  r0, r0, 4
        SBCO r0, C4, 4, 4

BEGIN:
        ledon
        MOV r3, DELAY_TIME

ON_TIME:
        SUB r3, r3, 1
        QBNE ON_TIME, r3, 0
        MOV r3, DELAY_TIME

TURN_OFF:
        ledoff

OFF_TIME:
        SUB r3, r3, 1
        QBNE OFF_TIME, r3, 0
CHECK_BUTTON:
        readbutton
        QBBS BEGIN, r2.t5
  
END:
        MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_0
        HALT


