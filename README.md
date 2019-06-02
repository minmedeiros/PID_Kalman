# PID & LQR Controller using Kalman for Parameter Estimation

This project is the implementation of PID-LQR controller sum on Arduino Uno. It uses the position measurement and estimates the Velocity and Electric Current using Kalman filter. Since this is a system with 3 internal variables and only one output (the measured position), the equations were adapted for this application in a way to optimize processing time.

## PID Gains

PID gains K_P, K_I and K_D were previously calculated in continous time and adapted for discrete time. It targets an arrival time of 120ms for 100% amplitude. 

## LQR Gains

LQR gains for each of the internal variables was calculated using Matlab function lqr after importing the state-space representation of the system.

## Kalman Filter

Kalman filter uses estimation error Q of 0.02 for all variables and measurement error R = 0.1 for the position. In addition, it is considering a 3x1 parameter H = [1 1 1]. This simplyfied a few equations and therefore is implicit in the code.

It calculates (based on earlier values) the variables represented by X = [current (A); velocity (rad/s); position (rad)] and the additional variable P, which adds the errors previously assumed. Then, it calculates Kalman Gain K of each of the three variables. Finally, the script estimates the next value of X, which is used by the LQR controller.

