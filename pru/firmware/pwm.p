.origin 0
.entrypoint SETUP

#define PRU1_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

#define GPIO2 0x481ac000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT   0x194
#define GPIO_DATAIN      0x138

#define LED_RED (1<<2)
#define CHANNEL_1 8
#define CHANNEL_2 10
#define CHANNEL_3 9
#define CHANNEL_4 11

.macro ledon
     MOV r4, GPIO2 | GPIO_SETDATAOUT
     MOV r6, LED_RED
     SBBO r6, r4, 0, 4
.endm

.macro ledoff
     MOV r4, GPIO2 | GPIO_CLEARDATAOUT
     MOV r6, LED_RED
     SBBO r6, r4, 0, 4
.endm


SETUP:
        // Enable the OCP master port
        LBCO &r0, C4, 4, 4
        CLR  r0, r0, 4
        SBCO r0, C4, 4, 4
         
READ_MEM:
        LBCO &r0, c24, 0, 32 // read the pulsewidth from SRAM

SET_ALL_HIGH:
        MOV r30, ((CHANNEL_4<<11) | (CHANNEL_2<<10) | (CHANNEL_3<<9) | (CHANNEL_1<<8)) // set all channels high
        ledon

HIGH_TIME:       
        SUB r0, r0, 1
        SUB r1, r1, 1
        SUB r2, r2, 1
        SUB r3, r3, 1
 
        QBNE CHECK_2, r0, 0
        CLR r30.t8

CHECK_2:
        QBNE CHECK_3, r1, 0
        CLR r30.t10

CHECK_3:
        QBNE CHECK_4, r2, 0
        CLR r30.t9

CHECK_4:
        QBNE HIGH_TIME, r3, 0
        CLR r30.t11

//only for testing purposes, needs to be replaced later
CHECK_BUTTON:
        ledoff
        MOV r5, GPIO2 | GPIO_DATAIN //for reading gpio bit
        LBBO r6, r5, 0, 4
        QBBS READ_MEM, r6.t5
         
END:
        MOV R31.b0, PRU1_R31_VEC_VALID | PRU_EVTOUT_0
        HALT


