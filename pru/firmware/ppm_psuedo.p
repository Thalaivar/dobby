origin 0
.entrypoint SETUP

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

    
OUTERLOOP:
       //CURRENT_CHANNEL = 1       
       //wait till pin becomes high:

INNERLOOP:

DELAY:
    SUB DELAY_reg, DELAY_reg, 1
    QBNE DELAY, DELAY_reg, 0       
       
       //wait till pin becomes high; whilst incrementing counter && counter<2000us
         // if counter> 2000:
               //OUTERLOOP
         //if pin_becomes high:
                // update_channel
                update_channel1:
                    MOV CHANNEL1_reg, COUNTER_reg
                    
                update_channel2:
                update_channel3:
                update_channel4:
                update_channel5:
                update_channel6:
                // update_register
                // INNERLOOP
        

