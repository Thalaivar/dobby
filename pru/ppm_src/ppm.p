.origin 0
.entrypoint SETUP

#define pulse_to_cycles 100
#define maxpulse 2100*(pulse_to_cycles)
#define minpulse 900*(pulse_to_cycles)
#define channel_size SIZE(.u32)

.struct Channels
	.u32	ch1
	.u32	ch2
	.u32	ch3
	.u32	ch4
	.u32	ch5
	.u32	ch6
.ends

.struct signalControl
    .u32 max_pulse
    .u32 min_pulse
.ends

.assign signalControl, r9, r10, signal
.assign Channels, r3, r8, channels

SETUP:
    MOV r2, 0  // set current channel val to 0
    MOV r1, 1  // set channel counter to 1st channel
    MOV signal.max_pulse, maxpulse // set max pulsewidth
    MOV signal.min_pulse, minpulse // set min pulsewidth
    // initialise signal states

WAITFORHIGH:
    QBBC WAITFORHIGH, r31.t15 // wait till pin goes high

READHIGH:
    ADD r2, r2, 1 // start counting up current channel value

    // channel value is time between two consecutive signal rises (or falls),
    // so we need to count the short signal high also
    QBBS READHIGH, r31.t15

    // now we have the high signal part of the channel value
    // so we need to read the low signal part


READLOW:
    ADD r2, r2, 1 // continue counting up current channel value
   
    // until signal goes high again, we need to count value
    QBBC READLOW, r31.t15
    
    // if read value is greater than 2000us, reset channel to 1
    QBLT RESET, r2, signal.max_pulse

    // if read value is less than 900us, its shit and wait for next pulse
    QBGT READHIGH, r2, signal.min_pulse
    
    // now r2 holds actual channel value
       
   
CHECK:
    	//start checks for which channel to write to
      QBEQ CHANNEL1, r1, 1
      QBEQ CHANNEL2, r1, 2
      QBEQ CHANNEL3, r1, 3
      QBEQ CHANNEL4, r1, 4
      QBEQ CHANNEL5, r1, 5
      QBEQ CHANNEL6, r1, 6

CHANNEL1:
      //save channel1 value
      MOV channels.ch1, r2
      //reset current channel value holder
      MOV r2, 0
      //increment channel counter
      ADD r1, r1, 1
      //start reading high signal part of next channel
      QBA READHIGH

CHANNEL2:
      //save channel2 value
      MOV channels.ch2, r2
      //reset current channel value holder
      MOV r2, 0
      //increment channel counter
      ADD r1, r1, 1
      //start reading high signal part of next channel
      QBA READHIGH

CHANNEL3:
      //save channel3 value
      MOV channels.ch3, r2
      //reset current channel value holder
      MOV r2, 0
      //increment channel counter
      ADD r1, r1, 1
      //start reading high signal part of next channel
      QBA READHIGH

CHANNEL4:
      //save channel4 value
      MOV channels.ch4, r2
      //reset current channel value holder
      MOV r2, 0
      //increment channel counter
      ADD r1, r1, 1
      //start reading high signal part of next channel
      QBA READHIGH

CHANNEL5:
      //save channel5 value
      MOV channels.ch5, r2
      //reset current channel value holder
      MOV r2, 0
      //increment channel counter
      ADD r1, r1, 1
      //start reading high signal part of next channel
      QBA READHIGH

CHANNEL6:
      //save channel6 value
      MOV channels.ch6, r2
      //reset current channel value holder
      MOV r2, 0
      //reset channel counter
      MOV r1, 1
      //load channel values into PRU DRAM
 		  SBCO &channels, c24, 0, SIZE(channels)
      //start reading high signal part of next channel (that would be the long pulse)
      QBA READHIGH

RESET:
    //reset current channel value holder
      MOV r2, 0
      //reset channel counter
      MOV r1, 1
      //start reading high signal part of first channel
      QBA READHIGH
