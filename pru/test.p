.origin 0
.entrypoint START

#define INS_PER_US 200
#define INS_PER_DELAY_LOOP 2

#define DELAY 50 * 1000 * (INS_PER_US / INS_PER_DELAY_LOOP)
#define PRU0_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

#define GPIO2 0x481ac000                // address of gpio2 bank
#define GPIO_CLEARDATAOUT 0x190         // clearing gpio registers
#define GPIO_SETDATAOUT   0x194         // setting gpio registers
#define GPIO_DATAOUT      0x138         // reading gpio registers

START:
     LBCO    r0, c4, 4, 4
     CLR     r0, r0, 4 
     SBCO    r0, c4, 4, 4
