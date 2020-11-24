# Building-an-Estimator 
Flying Car: 3D Estimation
# Overview:
The 3D Estimation project is the final project in the Udacity Flying Car Nanodegree. In
this project, our goal was to implement a state estimator using an Extended Kalman
Filter (EKF) to fuse multiple sources of sensor data (GPS, magnetometer, IMU) from a
simulated quadrotor to estimate the vehicle's state in 3 dimensions.
# The following sensors will be used:
•GPS: 3D position, 3D velocity
•IMU (accelerometer and gyroscope): 3D acceleration, angular rotation rates
•Magnetometer: heading (yaw angle)
# Project Steps:
# Step 1: Sensor Noise
For the controls project, the simulator was working with a perfect set of sensors,
meaning none of the sensors had any noise. The first step to adding additional realism
to the problem, and developing an estimator, is adding noise to the quad's sensors.
For the first step, you will collect some simulated noisy sensor data and estimate the
standard deviation of the quad's sensor.
1.Run the simulator in the same way as you have before
2.Choose scenario 06_NoisySensors. In this simulation, the interest is to record
some sensor data on a static quad, so you will not see the quad move. You will
see two plots at the bottom, one for GPS X position and one for The
accelerometer's x measurement. 
3.Process the logged files to figure out the standard deviation of the the GPS X
signal and the IMU Accelerometer X signal.
4.Plug in your result into the top of config/6_Sensornoise.txt. Specially, set the values
for MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY to be the values you have
calculated.
5.Run the simulator. If your values are correct, the dashed lines in the simulation
will eventually turn green 
Success criteria: Your standard deviations should accurately capture the value of
approximately 68% of the respective measurements.
# Result
I imported the sensor data from config/log/Graph1.txt (GPS X Data)
and config/log/Graph2.txt (Accelerometer X Data) to an excel spreadsheet and ran the
built in standard deviation function on each dataset to determine the standard
deviation of each sensor. This calculation can be found in Graph1.xlsx and Graph2.xlsx
The calculated standard deviations correctly match those found
in config/SimulatedSensors.txt of 0.7 for the GPS sensor and 0.5 for the accelerometer.

