.origin 0
.entrypoint READ_MEM

#define PRU1_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

#define GPIO2 0x481ac000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT   0x194
#define GPIO_DATAIN      0x138

//pwm frequency is 50Hz so loop time is 20000uS
#define REFRESH_TIME 20000

//calculated time for each loop after
//pins are set high is 120ns
//so for X uS pulse on channel 1, r0 must
//be 1000*X/120 = (100/12)X
#define EQUIV_CYCLES 100/12

READ_MEM:
        LBCO &r0, c24, 0, 32 // read the pulsewidth from PRU1 DRAM
        MOV r30.b1, 0x0F     //set all 4 channels high
        MOV r4, REFRESH_TIME*EQUIV_CYCLES

// REGISTER -------- WHAT DOES IT CONTAIN
//    0     -------- High cycles for channel 1
//    1     -------- High cycles for channel 2
//    2     -------- High cycles for channel 3
//    3     -------- High cycles for channel 4
//    4     -------- Total cycles needed for 50Hz PWM

//only start sending the next pulsewidth after REFRESH_TIME is over
//for this we have to delay for some time till a total of REFRESH_TIME uS have
//passed.

// LOOP          ---------- TIME (in nS)
// INNER_LOOP    ----------     45
// DELAY_OFF     ----------     10

INNER_LOOP:
        SUB r0, r0, 1
        SUB r1, r1, 1
        SUB r2, r2, 1
        SUB r3, r3, 1
        SUB r4, r4, 1
        QBNE CHECK2, r0, 0 //check if channel one has been high for required pulsewidth
        CLR r30.t8          //if it has, set it low

CHECK2:
        QBNE CHECK3, r1, 0 //check if channel two has been high for required pulsewidth
        CLR r30.t10         //if it has, set it low

CHECK3:
        QBNE CHECK4, r2, 0 //check if channel three has been high for required pulsewidth
        CLR r30.t9          //if it has, set it low

CHECK4: 
        //check if channel four has been high for required pulsewidth
        QBNE INNER_LOOP, r3, 0  //if not, go back to start of inner loop         
        CLR r30.t11             //if it has, set it low

DELAY_OFF:
        //delay till REFRESH_TIME is over
        SUB r4, r4, 1
        QBNE DELAY_OFF, r4, 0
        QBA READ_MEM
