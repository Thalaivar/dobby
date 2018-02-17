.origin 0
.entrypoint READ_MEM

#define PRU1_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

READ_MEM:
        LBCO &r0, c24, 0, 32 // read the pulsewidth from SRAM

WRITE_MEM:
        SBCO &r0, c24, 0, 4 
        SBCO &r1, c24, 1, 4
        SBCO &r2, c24, 2, 4
        SBCO &r3, c24, 3, 4

END:
        MOV   r31.b0, PRU1_R31_VEC_VALID | PRU_EVTOUT_0 
        HALT
