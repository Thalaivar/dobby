origin 0
.entrypoint SETUP

//pseudocode for writing ppm receiver
//major tasks:
//  pin_declaration 
//  setup of ppm
//  reading ppm
// writing values to register for main script to access


OUTERLOOP:
       //CURRENT_CHANNEL = 1       
       //wait till pin becomes high:
       // if high: INNERLOOP
       
       INNERLOOP:
       //   wait for ~1000us
       
       //wait till pin becomes high; whilst incrementing counter && counter<2000us
         // if counter> 2000:
               //OUTERLOOP
         //if pin_becomes high:
                // update_channel
                // update_register
                // INNERLOOP
        

