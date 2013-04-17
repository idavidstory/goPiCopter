goPiCopter
==========

Control a Quadcopter with a RaspberryPi, using the Go programming language.

Controller parts I've purchased already
Cost Weight Description
 $35   45g  Raspberry Pi model B
 $12    2g  USB wifi adapter
 $25    2g  adafruit's L3GD20 Triple Axis Gyroscope
 $25    2g  adafruit's LSM303 Triple Axis Accelerometer / Magnetometer(compass)
 $15    6g  adafruit's PCA9685 16 Channel 12-bit PWM / Servo driver
---- -----
$112   57g

Quadcopter parts I need to purchase (this is my first guess, may change soon)
Cost Weight Description
 $12  280g  Q450 Glass Fiber Quadcopter Frame 450mm (~ 17 inches)
 $44  188g  (4) HobbyKing 30A Blue Series Brushless Speed Controller
$128  316g  (4) NX-4008-620kv Brushless Quadcopter Motor 620RPM 30A 180W 79g
  $3   45g  (4) 8045 Propellers 2CW/2CCW
 $33  335g  Turnigy 4000mAh 35C Lipo Pack (nano-tech)
---- -----
$220 1164g


---- -----
$332 1221g  Combined, thats approximately 2.7 pounds

I'm sure you can do it cheaper and lighter, I started out just purchasing
the sensors to go with my Raspberry Pi to see if I could write a controller
I'd be happy with.  I'm getting closer, but it is still a work in progress.

The IMU tells you where you are at (Yaw,Pitch,Roll).
I need to write a flight controller that takes
where you're at, and applys the desired direction
from the Remote Control to determine how much
power to apply to the motors.

For the Remote Control, I'm thinking a simple interface on a phone/tablet
that communicates to the quadcopter via wifi.

