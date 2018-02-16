.origin 0
.entrypoint SETUP

#define PRU1_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

#define GPIO2 0x481ac000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT   0x194
#define GPIO_DATAIN      0x138

SETUP:
        // Enable the OCP master port
        LBCO r0, C4, 4, 4
        CLR  r0, r0, 4
        SBCO r0, C4, 4, 4
         
READ_MEM:
        lbco &r0, c24, 0, 4 // read the pulsewidth from SRAM
        SET r30.t8          // set SERVO1 high

ON_LOOP:
        SUB r0, r0, 1
        QBNE ON_LOOP, r0, 0
        CLR r30.t8
        MOV r1, 100000

DELAY:

        SUB r1, r1, 1
        QBNE DELAY, r1, 0

CHECK_BUTTON:
        MOV r5, GPIO2 | GPIO_DATAIN //for reading gpio bit
        LBBO r6, r5, 0, 4
        QBBS READ_MEM, r6.t5
         
END:
        MOV R31.b0, PRU1_R31_VEC_VALID | PRU_EVTOUT_0
        HALT


