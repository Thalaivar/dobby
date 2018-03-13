dobby
======

Implementation of an autonomous quadrotor drone based on SMC and fuzzy control
---------------------------------------------------------------------------------------

Chosen hardware : BeagleBone Blue

Tasks to do:
------------

* ~~Write MPU9250 driver.~~
* Write BMP180 driver.
* ~~Write Motor driver.~~
* Write AHRS algorithm. (delegated to the DMP, not enough skill to write own algorithm)
* ~~PRU PPM and PWM libraries.~~
* ~~Write accel_calibration function.~~
* ~~Write gyro_calibration function~~
* ~~Write mag_calibration function~~
* Write failsafes and arm checks
* Test the ~~PPM~~ and PWM outputs

Status:
--------
Still writing up the low level controller code base.

Next major test:
----------------
Attitude Controller testing
