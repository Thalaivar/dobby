dobby
======

Implementation of an autonomous quadrotor drone based on SMC and fuzzy control
---------------------------------------------------------------------------------------

Chosen hardware : BeagleBone Blue

Tasks to do:
------------

* Write AHRS algorithm. (delegated to the DMP, not enough skill to write own algorithm)
* Write failsafes and arm checks
* Map thrusts to PWM
* Get Inertia estimate
* Fix loop times

Status:
--------
Currently on the test bench.

Next major test:
----------------
Attitude Controller testing
