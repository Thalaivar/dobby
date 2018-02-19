origin 0
.entrypoint SETUP

//pseudocode for writing ppm receiver
//major tasks:
//  pin_declaration 
//  setup of ppm
//  reading ppm
// writing values to register for main script to access

//
//loop{
//  start counter when high is received,
//  store counter_value when low,
//  reset counter to zero,
// }



LOOP:
    //read_pin
    //wait till high: if high: wait for ~1000us
        //keep reading_pin 
        //increment counter if pin is low
        // if pin becomes high
        // counter+1000 is channel_value
        // goto LOOP

