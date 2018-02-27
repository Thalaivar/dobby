.origin 0
.entrypoint SETUP

#define cycles_to_pulse 10
#define maxpulse 2100*cycles_to_pulse
#define minpulse 900*cycles_to_pulse

.struct Channels
	.u32	ch1
	.u32	ch2
	.u32	ch3
	.u32	ch4
	.u32	ch5
  .u32	ch6
.ends

.assign Channels, r4, r9, channel_vals

SETUP:
    MOV r3, 0  // set current channel val to 0
    MOV r2, 1  // set channel counter to 1st channel

WAITFORHIGH:
    QBBC WAITFORHIGH, r31.t14  //wait till pin goes high

READ:
     ADD r3, r3, 1 //count up r3 till pin goes low
     
     //start checks for which channel to write to
     QBEQ CHANNEL1, r2, 1
     QBEQ CHANNEL2, r2, 2
     QBEQ CHANNEL3, r2, 3
     QBEQ CHANNEL4, r2, 4
     QBEQ CHANNEL5, r2, 5
     QBEQ CHANNEL6, r2, 6

     //if channel read was last channel
     QBGT RESET,    r2, max_pulse
