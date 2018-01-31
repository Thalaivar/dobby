# ARMCopter
Implementation of sliding mode control on a quadrotor


Chosen hardware (obviously):    BeagleBone Blue

Current to-do list:
        1) Make the motors library and include arming, output, etc.
        2) Comms. with GCS, to integrate MAVLink or not?
        3) Data logging
        4) pre-arm checks to be made, also failsafes
        5) AHRS to be set up along with filtering
        6) **Need to evaluate the real time capabilities of PREEMPT_RT kernel currently on BeagleBone.** Learrning and          impleminting pthreading and priority control?
