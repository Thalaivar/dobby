.origin 0
.entrypoint READ_MEM


// REGISTER -------- WHAT DOES IT CONTAIN
//    4     -------- High cycles for channel 1
//    5     -------- High cycles for channel 2
//    6     -------- High cycles for channel 3
//    7     -------- High cycles for channel 4
//    8     -------- Total cycles needed for one period, will be set by user

.struct Channels
	.u32	ch1	// range 0 - max_pulse
	.u32	ch2	// range 0 - max_pulse
	.u32	ch3	// range 0 - max_pulse
	.u32	ch4	// range 0 - max_pulse
	.u32	max_pulse	// maximum pulsewidth i.e. period
.ends

 .assign Channels, r4, r8, channels

READ_MEM:
        LBCO &channels, c24, 0, SIZE(channels) // read the pulsewidth from PRU1 DRAM
        MOV r30.b1, 0x0F     //set all 4 channels high
        MOV r0, 0            //r0 will count up till max_pulse cycles have passed

INNER_LOOP:
        QBNE CHECK1, r0, channels.ch1 //check if since start pulsewidth
                                      //equal to CH1 has been sent
        CLR r30.t8                    //if it has, set CH1 low

CHECK1:
        QBNE CHECK2, r0, channels.ch2 //check if since start pulsewidth
                                      //equal to CH2 has been sent
        CLR r30.t10         //if it has, set CH2 low

CHECK2:
        QBNE CHECK3, r0, channels.ch3 //check if since start pulsewidth
                                      //equal to CH3 has been sent
        CLR r30.t9          //if it has, set it low

CHECK3:
        QBNE FINALCHECK, r0, channels.ch4 //check if since start pulsewidth
                                      //equal to CH4 has been sent
        CLR r30.t11          //if it has, set it low

FINALCHECK:
        QBEQ READ_MEM, r0, channels.max_pulse //if total period is over, read new values in
        ADD r0, r0, 1      //if total period is not over, count up r0
        QBA INNER_LOOP
