.origin 0
.entrypoint SETUP

#define cycles_to_pulse 10
#define maxpulse 2100*cycles_to_pulse
#define minpulse 900*cycles_to_pulse
#define channel_size SIZE(.u32)

.struct Channels
	.u32	ch1
	.u32	ch2
	.u32	ch3
	.u32	ch4
	.u32	ch5
	.u32	ch6
.ends

.assign Channels, r4, r8, channels

SETUP:
    MOV r3, 0  // set current channel val to 0
    MOV r2, 1  // set channel counter to 1st channel

WAITFORHIGH:
     QBBC WAITFORHIGH, r31.t14  //wait till pin goes high

SHORTHIGH:
		 QBBS SHORTHIGH, r31.t14   //do nothing till pin goes low again

READ:
		 //count up r3 till pin goes high again
     ADD r3, r3, 1
     QBBC READ, r31.t14

		 //if read value is greater than 2000us, reset channel to 1
		 QBGT RESETCHANNEL, r3, maxpulse

		 //if read value is less than 900us, its shit and wait for next pulse
		 QBLT WAITFORHIGH, r3, minpulse

CHECK:
		 //start checks for which channel to write to
     QBEQ CHANNEL1, r2, 1
     QBEQ CHANNEL2, r2, 2
     QBEQ CHANNEL3, r2, 3
     QBEQ CHANNEL4, r2, 4
     QBEQ CHANNEL5, r2, 5
     QBEQ CHANNEL6, r2, 6

CHANNEL1:
			//save channel1 value
			MOV channels.ch1, r3
			//increment channel counter
			ADD r2, r2, 1
			//wait till pulse position goes back low to start reading next channel
			QBA SHORTHIGH

CHANNEL2:
			//save channel2 value
			MOV channels.ch2, r3
			//increment channel counter
			ADD r2, r2, 1
			//wait till pulse position goes back low to start reading next channel
			QBA SHORTHIGH

CHANNEL3:
			//save channel3 value
			MOV channels.ch3, r3
			//increment channel counter
			ADD r2, r2, 1
			//wait till pulse position goes back low to start reading next channel
			QBA SHORTHIGH

CHANNEL4:
			//save channel4 value
			MOV channels.ch4, r3
			//increment channel counter
			ADD r2, r2, 1
			//wait till pulse position goes back low to start reading next channel
			QBA SHORTHIGH

CHANNEL5:
			//save channel5 value
			MOV channels.ch5, r3
			//increment channel counter
			ADD r2, r2, 1
			//wait till pulse position goes back low to start reading next channel
			QBA SHORTHIGH

CHANNEL6:
			//save channel6 value
			MOV channels.ch6, r3
			//reset channel counter
			MOV r2, 1
			//wait till pulse position goes back low to start reading next channel
			QBA SHORTHIGH

RESET:
		 //load channel values into PRU DRAM
		 SBCO &channels, c24, 0, SIZE(channels)
		 //set channel counter to 1
		 MOV r2, 1
		 //wait for channel1 to start
		 QBA WAITFORHIGH
