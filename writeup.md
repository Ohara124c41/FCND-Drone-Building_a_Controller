# The C++ Project Write-Up #


## Provide a write-up ##

##### Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. #####

You're reading it!

## Implemented Controller ##

### Implemented body rate control in C++. ###
Use V3F and multiply the inertial term to get the moment.

```c
V3F Inertia;
Inertia.x = Ixx;
Inertia.y = Iyy;
Inertia.z = Izz;
momentCmd = Inertia * kpPQR * (pqrCmd - pqr);
```

### Implement roll pitch control in C++. ###

It is import to note that the tilt angles are going to be important here, for when we want to balance the drone in the simulator (or field). The different constraints can be separated into the x and y components. Note, floats are used over doubles here. Though floating points are less precise, they can save memory and emulate "real-time" behavior easier in the simulator. This is a good practice for drones in general, since they are technically an embedded system.

```c
if (collThrustCmd > 0) {
  float c = - collThrustCmd / mass;

  float bx_commanded = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
  float bx_error = bx_commanded - R(0,2);
  float bx_p = kpBank * bx_error;

  float by_commanded = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
  float by_error = by_commanded - R(1,2);
  float by_p = kpBank * by_error;

  pqrCmd.x = (R(1,0) * bx_p - R(0,0) * by_p) / R(2,2);
  pqrCmd.y = (R(1,1) * bx_p - R(0,1) * by_p) / R(2,2);

} else {
  pqrCmd.x = 0.0;
  pqrCmd.y = 0.0;
}

pqrCmd.z = 0;

```
### Implement altitude controller in C++. ###
The u-bar term is needed to acquire the thrust. Convert the formula into an algorithm that can acheive this.
```c

float z_error = posZCmd - posZ;
float kp_z = kpPosZ * z_error;
float z_dot_error = velZCmd - velZ;
integratedAltitudeError += z_error * dt;


float kd_z = kpVelZ * z_dot_error + velZ;
float ki_z = KiPosZ * integratedAltitudeError;
float b_z = R(2,2);

float u1_bar = kp_z + kd_z + ki_z + accelZCmd;

float acc = (u1_bar - CONST_GRAVITY) / b_z;

thrust = - mass * CONSTRAIN(acc, - maxAscentRate / dt, maxAscentRate / dt);


```
### Implement lateral position control in C++. ###
Just expanding from the source material, we can concisely implement the lateral position control.

```c
velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

V3F commanded = kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel) + accelCmd;

accelCmd.x = CONSTRAIN(commanded.x, -maxAccelXY, maxAccelXY);
accelCmd.y = CONSTRAIN(commanded.y, -maxAccelXY, maxAccelXY);
accelCmd.z = 0;

```

### Implement yaw control in C++. ###
This is pretty straight forward, based on everything else we have done. Don't you love when only one line is needed to complete a function?

```c
yawRateCmd = kpYaw * (yawCmd - yaw);

```
### Implement calculating the motor commands given commanded thrust and moments in C++. ###
This should probably have been at the top of the writeup, but this is fine. The algorithm converts the forces as a funtion of tau, w.r.t the arm length and kappa.

```c
float arm = L / sqrtf(2.f);

float t0 = momentCmd.x / arm;
float t1 = momentCmd.y / arm;
float t2 = - momentCmd.z / kappa;
float t3 = collThrustCmd;

cmd.desiredThrustsN[0] = (t0 + t1 + t2 + t3)/4.f;  // front left
cmd.desiredThrustsN[1] = (-t0 + t1 - t2 + t3)/4.f; // front right
cmd.desiredThrustsN[2] = (t0 - t1 - t2 + t3)/4.f ; // rear left
cmd.desiredThrustsN[3] = (-t0 - t1 + t2 + t3)/4.f; // rear right
```
## Flight Evaluation ##

All Scenarios were passed with the following results:
```c

# Scenario 1
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds

# Scenario 2
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

# Scenario 3
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds

# Scenario 4
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds

# Scenario 5
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds

```
### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory. ###


Let's begin! This is what they gave me.

![00](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/00.gif?raw=true)

Well, they say "fail fast", and I suppose this is a prime example of that!

![07](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/07.gif?raw=true)

Alright! It looks like Scenario 1 is working properly!

![01](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/03.gif?raw=true)

Moving on to Scenario 2 looks like a scene featuring your favorite space pirate.

![08](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/08.gif?raw=true)

Alright, the behavior is as intended. Let's move on!

![02](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/04.gif?raw=true)

Smooth dual-drone flight here! Scenario 3 = complete.

![03](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/01.gif?raw=true)

Time for the real fine-tuning of parameters. Oh wait, where are you going?! Note: The red drone (quad1) is the heaviest, yellow (quad2) is ideal, and the green drone (quad3) has its CoM shifted to the backside.

![06](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/06.gif?raw=true)

This is about as smooth as I am ever going to get three drones to work simultaneously...
![04](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/02.gif?raw=true)

Time for some object tracking! Quad2 has some very stable behavior and passes the specifications.

![05](https://github.com/Ohara124c41/FCND-Drone-Building_a_Controller/blob/master/animations/05.gif?raw=true)


### Notes ###

* A lot of parameters can be calculated using the formulae. However, sometimes better results can acheived by trial and error.
* Getting the scenarios to pass is great, but it is better spend some time optimizing the performance and behavior of the drones.  Therefore, the goal should be for smooth flight, minimizing the oscillations on the wave-form generator and minimizing t_set.
* Trade-offs will always be present in this "single input, multiple device" type parallelism. Aiming for overall robust behavior between drones is better than optimizing a single done.
* This project is actually a lot of fun!
