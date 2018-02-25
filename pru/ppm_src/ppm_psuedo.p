.origin 0
.entrypoint SETUP

#define PRU1_R31_VEC_VALID 32
#define PRU_EVTOUT_0 3

// add the registers for corresponding channels
#define CHANNEL_1_reg
#define CHANNEL_2_reg
#define CHANNEL_3_reg
#define CHANNEL_4_reg
#define CHANNEL_5_reg
#define CHANNEL_6_reg
#define COUNTER_reg
#define CURRENT_CHANNEL

#define DELAY_TIME  //represents the delay time(1000us)..... define as a number 
#define DELAY_reg   //represents the counter pin for delay function
#define PPMin

#define MEMORY_REGISTER

SETUP:
    MOV DELAY_reg, DELAY_TIME
    MOV COUNTER_reg, 0
    
OUTERLOOP:
    MOV CURRENT_CHANNEL, 1
    QBNE OUTERLOOP, PPMin, 1 //waiting till pin becomes high

INNERLOOP:

DELAY: //we wait for ~1000us
    SUB DELAY_reg, DELAY_reg, 1
    QBNE DELAY, DELAY_reg, 0       

ICE:
    ADD COUNTER_reg, COUNTER_reg, 1
    QBGT OUTERLOOP, COUNTER_reg, 2000 //reset ppm channel to 1
    QBNE ICE, PPMin, 1 //waiting and incrementing till pin high

UPDATE_CHANNEL:
    QBEQ UPDT_CHN1, CURRENT_CHANNEL, 1
    QBEQ UPDT_CHN2, CURRENT_CHANNEL, 2
    QBEQ UPDT_CHN3, CURRENT_CHANNEL, 3
    QBEQ UPDT_CHN4, CURRENT_CHANNEL, 4
    QBEQ UPDT_CHN5, CURRENT_CHANNEL, 5
    QBEQ UPDT_CHN6, CURRENT_CHANNEL, 6

UPDT_CHN1:    
    MOV CHANNEL_1_reg, COUNTER_reg
    MOV COUNTER_reg, 0
    QBA INNERLOOP

UPDT_CHN2:    
    MOV CHANNEL_2_reg, COUNTER_reg
    MOV COUNTER_reg, 0
    QBA INNERLOOP

UPDT_CHN3:    
    MOV CHANNEL_3_reg, COUNTER_reg
    MOV COUNTER_reg, 0
    QBA INNERLOOP

UPDT_CHN4:    
    MOV CHANNEL_4_reg, COUNTER_reg
    MOV COUNTER_reg, 0
    QBA INNERLOOP

UPDT_CHN5: 
    MOV CHANNEL_5_reg, COUNTER_reg
    MOV COUNTER_reg, 0
    QBA INNERLOOP

UPDT_CHN6:    
    MOV CHANNEL_6_reg, COUNTER_reg
    MOV COUNTER_reg, 0
    QBA WRITE_REG

WRITE_REG:
    SBCO  //Need to add the registers here
    QBA OUTERLOOP

END:
        MOV R31.b0, PRU1_R31_VEC_VALID | PRU_EVTOUT_0
        HALT

