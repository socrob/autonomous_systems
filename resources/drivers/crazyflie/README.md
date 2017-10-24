crazy flie
===

Conversation with Romulo Texeira, PhD student at tecnico that worked with the crazy flie robot

Oscar: Does the crazyfly has IMU? and if so, how good is it?

Romulo: It has an imu. Angular information is quite good.
But you can't use accelerometer to estimate position not velocity for more than few milisecs.

Oscar: why?

Romulo: in general these cheap imu provides very noisy acceleration reading.
Therefore you integrate a lot of noise to obtain position and velocity.
Take a look  on this table http://www.chrobotics.com/library/accel-position-velocity
A good imu costs few thousands euros. A really good one I think cost many thousand euro.
Crazyflie pack cost was around 200 euro.

Oscar: what about the camera calibration

Romulo: You get a lot of noise and distorsion from the crazyflie, I suggest
you to crop the image of the crazyflie and calibrate based on a subsection of it.

Romulo: The pkgs that I developed are under indigo, there is a laptop with all setup ready.
I really recommend you to use the laptop in which all the setup is ready
There is a student working on that currently, you can ask him.

I think Meysam is the ideal pilot for the drone, is difficult to fly.
Calibrate the system (mocap) just before the experiment.

links
===

Crazy flie website:

        https://www.bitcraze.io/crazyflie-2/

ROS wrapper that they use:

        https://github.com/whoenig/crazyflie_ros
