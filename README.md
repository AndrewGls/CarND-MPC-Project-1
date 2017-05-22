# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Build Instruction
I have implemented a simple visualization using OpenCV, which is displayed in a separate window.
To build the project with the visualization enabled, do:
```
mkdir build; cd build
cmake .. -DENABLE_VISUALIZATION=ON
make
```

## Video
A video of the car completing the course at approx. 40 mph, including visualization of the control behavior
can be found at: https://youtu.be/E2ix3uLOxtM


## The Model
The vehicle state has the following components:

- x: position of the vehicle in the forward direction
- y: position of the vehicle in the lateral direction
- psi: yaw angle or orientation of the vehicle
- v: speed of the vehicle

The actuators of the vehicle are:

- delta: steering angle in radians
- a: acceleration

The update equations are used to compute the state of the car at the next time step, based on the state at the
current time step:

```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta * dt
v_t+1 = v_t + a_t * dt
```

In addition to the state and actuator variables, they include the time step `dt` and the factor
`Lf` which is defined as the distance between the vehicle's front and its center of gravity. `Lf`, 
together with the current velocity `v_t`, determine the current turning radius of the vehicle.


## Timestep Length and Frequency
I started out with N=25 steps and time step length of dt=50ms which already provided good results. I noticed, however,
that this combination of parameters would lead to numerically unstable results in sharp turns at higher speeds.
Here, the solver would find a solution with a much too aggressive steering angle for some time steps, which would result
in the car veering off the track.

I then tried reducing the number of steps and simultaneously increasing the length of
the time steps. Reducing the number of steps also has the advantage of greatly improving the performance of the system,
as the numerical fitting of a solution is computationally expensive.

I settled on a combination of
N = 10 and dt = 100 ms which gives good results at moderate speeds (e.g up to 40 mph), when combined with a carefully selected 
penalty weight for the steering angle (see below). For very high speeds, I found that a value N = 15 gave better results, 
because it becomes increasingly important to look further ahead to plan a trajectory with increasing
speed.

In essence, the number of steps is chosen as a compromise between reacting to the current
curvature of the road, versus looking ahead to plan the ideal trajectory. Both looking ahead too far _and_ looking
only at the current curvature will lead to undesired control behavior. All of this is constrained
further by the computational requirements of the fitting algorithm.


## Polynomial Fitting and MPC Preprocessing
In a first step, the waypoints are transformed into the vehicle coordinate
system. The resulting x-direction is the forward direction of the vehicle, while the y-direction represents the lateral
displacement of waypoints relative to the center of the vehicle. This allows for easy fitting of a polynomial of the form 
`y = a*x^3 + b*x^2 + c*x + d`. When constructing the initial state vector for the solver, the values for x, y, and psi can
now be conveniently set to zero because the coordinate system is defined relative to the vehicle. The initial cross track error is 
equal to the constant term of the fitted polynomial in this coordinate system, and the initial orientation error is
calculated from the first derivative of the polynomial at x=0.


## Model Predictive Control with Latency
The additional latency introduced in `main.cpp` presents a problem: The steering value computed by the solver
for the first time-step will already be in the past, which will lead to an increase of the CTE and orientation error. 
The solver will try to compensate the resulting increase in cost in the next iteration but this compensation will
also come too late. When this process is repeated a few times, the vehicle will start to oscillate around the intended
trajectory, especially at higher velocities.

In order to mitigate this problem, I implemented an averaging procedure that takes the mean of the first three predicted
steering angles. With a time step of 100ms, this corresponds a time interval of 300ms, which is about twice of the total
expected latency. This averaging measure successfully stabilizes the control behavior.


## Cost Function and Penalty Factors for the Steering Angle and Acceleration
In order to solve for a possible trajectory, we define a cost function that constrains our solution. It has the form:

```
Cost = cte^2 + e_psi^2 + e_vel^2 + w_delta * delta^2 + w_a * a^2 + d_delta^2 + d_a^2 
```

The first three terms penalize the cross track error CTE, the orientation error e_psi and the velocity error.
By minimizing these error terms, the vehicle will try to stay on the planned waypoint trajectory with the given
orientation and speed. The last two term penalize sudden changes in the actuators which would lead to undesired 
sudden accelerations for the passengers. The two terms in the middle penalize the absolute value of the actuators.
The term `w_delta * delta^2` penalizes the absolute value of the steering angle and is crucial for optimizing the
control behavior. For small values of the penalty factor `w_delta`, the controller will try to follow the waypoint
trajectory very closely, which will sometimes lead to unintended steering commands, especially when sharp turns
are traversed at high speeds.

I found that a value of `w_delta = 125` gave good result for medium velocities up to about 40 mph, which
should be a reasonable maximum speed for a safe trip around the course with a passenger car.
The car stays close to the center of the road at all times, including in sharp turns.

I also successfully tried to increase the speed up to 80 mph. Of course, the vehicle will now have to steer differently in order to complete
the sharp turns at this velocity. The controller will have to look ahead farther and cut the corners at the inside of the 
curves like a race driver. This is accomplished by drastically increasing `w_delta` which will effectively restrict
the steering angle to more closely follow the ideal racing trajectory instead of the center of the road. I found that
a value of `w_delta = 50000` together with an increase to N=15 enabled the car to stay on the course at this velocity.
See https://youtu.be/fUuCiLD4Zfo for a video.

I found that a penalty factor for the acceleration of `w_a = 2` helped to prevent the controller from adjusting the
throttle too aggressively which led to a constant breaking and acceleration cycle around the target velocity.