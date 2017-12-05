# Overview
This repository contains the implementation of PID control in Udacity's Self-Driving Car Nanodegree. I implemented a PID controler by following the instructions provided in the course. My implementation meets the Success Criteria. 

## Project Introduction
The car is supposed to follow a desired path in the simulator. The PID controler is using cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

## Success Criteria
My implementation meets the success criteria:
1. The vehicle successfully drives around the track.
2. No tire leaves the drivable portion of the track surface.
3. The car doesn't pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). 

## Running the Code
I used docker to setup the required environment. Once run_term2_docker.sh is executed, the main program can be built and ran by doing the following from the /src directory:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

## Implementation
I followed the instructions provided in the course to implement the controler. I have also used some hyperparameter to fine tune my implementation so my controller results in smooth motions:

1. A speed factor is applied to P, I, D based on the actual speed at each iteration. 
2. CTE and delta_CTE are averaged over two itteration to prevent high jerk in calculated steering angles.
3. P, I, D are optimized by using twiddle algorithm.

### Inputs to the controller
Here are the inputs to the PID loop:
1. cross track error (CTE)
2. velocity (mph)

### Effect of the parameters:
1. Term P is proportional to the current value of the error
2. Term I accounts for past values of the error and integrates them over time to produce the I term.
3. Term D is a best estimate of the future trend of the error, based on its current rate of change.

### Tuning of the parameters:
I tunned the parameters in 2 steps: 

a. In the first step, I manually tuned the parameters based on the following instruction so the vehicle can successfully drives around the track: 

1. Set all gains to zero.
2. Increase the P gain until the response to a disturbance is steady oscillation.
3. Increase the D gain until the the oscillations go away (i.e. it's critically damped).
3. Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
4. Set P and D to the last stable values.
5. Increase the I gain until it brings you to the setpoint with the number of oscillations desired

b. In the secondstep, I fine tuned the parameters by using Twiddle algorithm.

## Discussion

My implementation follows the general processing flow as taught in the lessons and I was able to meet the required performance. The interesting part was fine-tuning the parameters. I developped an algorithm to evaluate the quality of the steering values over 1 full track. The algorithm uses steering values and steering_dot and steering_dot_dot to generate a metric that I can use twiddle algorithm.

