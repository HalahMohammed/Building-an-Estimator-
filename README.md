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

# Result
Here I used the FromEuler123_RPY function of the supplied Quarternion class to create a
Quaternion from the passed in estimated roll, pitch and yaw state values.
I then used the supplied IntegrateBodyRate() function to integrate the Quaternion values
with the data from the IMU in the body frame.Then we call
the Pitch(), Roll() and Yaw() functions to generate predicted pitch, roll and yaw values in
the Quaternion. Finally I normalised the predicted Yaw between -pi and pi.
The code for this implementation can be found in src/QuadEstimatorEKF.cpp on lines 100 -
108.
Running this scenario resulted in the following output from std out terminal:
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
Step 3: Prediction Step
In this next step you will be implementing the prediction step of your filter.
1.Run scenario 08_PredictState. This scenario is configured to use a perfect IMU
(only an IMU). Due to the sensitivity of double-integration to attitude errors,
we've made the accelerometer update very insignificant
(QuadEstimatorEKF.attitudeTau = 100). The plots on this simulation show element of
your estimated state and that of the true state. At the moment you should see
that your estimated state does not follow the true state.
2.In QuadEstimatorEKF.cpp, implement the state prediction step in
the PredictState() functon. If you do it correctly, when you run
scenario 08_PredictState you should see the estimator state track the actual state,
with only reasonably slow drift, as shown in the figure below:
3.Now let's introduce a realistic IMU, one with noise. Run scenario 09_PredictionCov.
You will see a small fleet of quadcopter all using your prediction code to
integrate forward. You will see two plots:
•The top graph shows 10 (prediction-only) position X estimates
•The bottom graph shows 10 (prediction-only) velocity estimates You will
notice however that the estimated covariance (white bounds) currently do
not capture the growing errors.
4.In QuadEstimatorEKF.cpp, calculate the partial derivative of the body-to-global
rotation matrix in the function GetRbgPrime(). Once you have that function
implemented, implement the rest of the prediction step (predict the state
covariance forward) in Predict().
5.Run your covariance prediction and tune the QPosXYStd and
the QVelXYStd process parameters in QuadEstimatorEKF.txt to try to capture the
magnitude of the error you see. Note that as error grows our simplified model
will not capture the real error dynamics (for example, specifically, coming from
attitude errors), therefore try to make it look reasonable only for a relatively
short prediction period (the scenario is set for one second). A good solution
looks as follows:
Looking at this result, you can see that in the first part of the plot, our covariance (the
white line) grows very much like the data.
If we look at an example with a QPosXYStd that is much too high (shown below), we can
see that the covariance no longer grows in the same way as the data.
Another set of bad examples is shown below for having a QVelXYStd too large (first) and
too small (second). As you can see, once again, our covariances in these cases no
longer model the data well.
Success criteria: This step doesn't have any specific measurable criteria being
checked.
# Result
The implementation of PredictState() can be found in QuadEstimatorEKF.cpp on lines 174 -
181.
To determine the accelerations in the intertial frame from the body frame I passed the
acceleration of the vehicle, in body frame (accel) to the supplied Rotate_BtoI() method
from the attitude Quaternion. Once we have the accelerations in the inertial frame we
can integrate them with the current state into an updated predicted state by
multiplying by the timestep dt.
Running scenario 08_PredictState the estimated state tracks the actual state, with
only reasonably slow drift, as shown in the figure above.
The implementation of GetRbgPrime() for step 4 can be found in QuadEstimatorEKF.cpp on
lines 209 - 223 according to the following formula:
The function returns the 3x3 matrix representing the partial derivative at the given
point.
Finally, I completed the Predict() function using the supplied Jacobian gPrime of the
transition function g. The code for this implementation can be found
in QuadEstimatorEKF.cpp on lines 269 - 285.
The transition function g is defined as:
And its Jacobian gPrime is:
Taking these inputs we can determine the covariance matrix according to the
extended Kalman filter (EKF) equation:
ekfCov = gPrime * ekfCov * gPrime.transpose() + Q
Retuning the QPosXYStd and the QVelXYStd parameters leads to us being able to capture
the magnitude of the error we see.
Step 4: Magnetometer Update
Up until now we've only used the accelerometer and gyro for our state estimation. In
this step, you will be adding the information from the magnetometer to improve your
filter's performance in estimating the vehicle's heading.
1.Run scenario 10_MagUpdate. This scenario uses a realistic IMU, but the
magnetometer update hasn’t been implemented yet. As a result, you will notice
that the estimate yaw is drifting away from the real value (and the estimated
standard deviation is also increasing). Note that in this case the plot is showing
you the estimated yaw error (quad.est.e.yaw), which is drifting away from zero as
the simulation runs. You should also see the estimated standard deviation of
that state (white boundary) is also increasing.
2.Tune the parameter QYawStd (QuadEstimatorEKF.txt) for the QuadEstimatorEKF so
that it approximately captures the magnitude of the drift, as demonstrated here:
3.Implement magnetometer update in the function UpdateFromMag(). Once
completed, you should see a resulting plot similar to this one:
Success criteria: Your goal is to both have an estimated standard deviation that
accurately captures the error and maintain an error of less than 0.1rad in heading for
at least 10 seconds of the simulation.
Result
Step 2 tuning the QYawStd parameter to 0.08 was found to approximately capture the
magnitude of the drift.
The implementation of UpdateFromMag() can be found in QuadEstimatorEKF.cpp on lines 339 -
345.
First we create the array hPrime according to:
Then, we assign the current yaw to the vector zFromX and normalise it to between -pi
and pi.
These variables are then passed to the Update() method of the UpdateFromMag() to
perform the update.
After this implementation the following message is reported in the std output when
the scenario is run:
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 66% of the time
Step 5: Closed Loop + GPS Update
1.Run scenario 11_GPSUpdate. At the moment this scenario is using both an ideal
estimator and and ideal IMU. Even with these ideal elements, watch the position
and velocity errors (bottom right). As you see they are drifting away, since GPS
update is not yet implemented.
2.Let's change to using your estimator by setting Quad.UseIdealEstimator to 0
in config/11_GPSUpdate.txt. Rerun the scenario to get an idea of how well your
estimator work with an ideal IMU.
3.Now repeat with realistic IMU by commenting out these lines
in config/11_GPSUpdate.txt:
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
4.Tune the process noise model in QuadEstimatorEKF.txt to try to approximately
capture the error you see with the estimated uncertainty (standard deviation) of
the filter.
5.Implement the EKF GPS Update in the function UpdateFromGPS().
6.Now once again re-run the simulation. Your objective is to complete the entire
simulation cycle with estimated position error of < 1m (you’ll see a green box
over the bottom graph if you succeed). You may want to try experimenting with
the GPS update parameters to try and get better performance.
Success criteria: Your objective is to complete the entire simulation cycle with
estimated position error of < 1m.
At this point, congratulations on having a working estimator!
# Result
The implementation of UpdateFromGPS() can be found in QuadEstimatorEKF.cpp on lines 310 -
317.
Here we set the identity matrix of hPrime and then update zFromX state vector from the
current ekfState vector and pass these variables to the Update() method.
The resulting output from the std output following the implementation is:
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
Step 6: Adding Your Controller
Up to this point, we have been working with a controller that has been relaxed to work
with an estimated state instead of a real state. So now, you will see how well your
controller performs and de-tune your controller accordingly.
1.Replace QuadController.cpp with the controller you wrote in the last project.
2.Replace QuadControlParams.txt with the control parameters you came up with in
the last project.
3.Run scenario 11_GPSUpdate. If your controller crashes immediately do not panic.
Flying from an estimated state (even with ideal sensors) is very different from 
flying with ideal pose. You may need to de-tune your controller. Decrease the
position and velocity gains (we’ve seen about 30% detuning being effective) to
stabilize it. Your goal is to once again complete the entire simulation cycle with
an estimated position error of < 1m.
Success criteria: Your objective is to complete the entire simulation cycle with
estimated position error of < 1m.
# Result
After retuning the controller parameters by an approx 30% relaxation the output from
the std output window is:
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
