# Estimation Project #

## My Solution ##
This is the readme for my project submission. The original project README has be moved to Project - RM.md for reference. 
This has been a challenging project and I enjoyed incorportating the controller from the previous project at the end. The math was challenging at times during the lessons but implementing it in the project framework was a great exercise.


### Step 1: Sensor Noise ###
I solved this one by importing the CSV files into Excel and applying the mean, standard deviation and variance formulas to the data to get the result. See image below. The rounded up values in `SimulatedSensors.txt` matched the result.

<p align="center">
   <img src="images/sensor1.jpg" width="600"/>
   </br></br>
   <img src="images/sensor2.jpg" width="600"/>
</p>

**Screenshot and console output:**

</br>
<p align="center">
   <img src="images/task1_img.jpg" width="300"/><br/>
   <img src="images/task1_success.jpg" width="600"/>
</p>

### Step 2: Attitude Estimation ###
This step required the implementation of a complementary filter. I chose to use the suggested Quaternions rather than the Eulers that were presented in the lectures to avoid creating another rotation Matrix. The built in `IntegrateBodyRate` function was very helpful as was the ability to easily convert back to Euler angles. My code follows. Credit to section 7.1.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for explaining the implementation.

```C++
   //use the state to define a quaternion
	Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));

	//Integrate the measurements
	qt.IntegrateBodyRate((V3D)gyro, dtIMU);

	//Convert back to Euler
	V3F predictedAttitude = (V3F)qt.ToEulerRPY();
	float predictedRoll = predictedAttitude.x;
	float predictedPitch = predictedAttitude.y;
	ekfState(6) = predictedAttitude.z;

	// normalize yaw to -pi .. pi
	if (ekfState(6) > F_PI) ekfState(6) -= 2.f * F_PI;
	if (ekfState(6) < -F_PI) ekfState(6) += 2.f * F_PI;
```
<br/><br/>
**Screenshot and console output**:</br>
<p align="center">
   <img src="images/step2_graph.jpg") width="300"/></br>
   <img src="images/step2_console.jpg") width="600"/>
</p>


### Step 3: Prediction Step ###
There were two parts to this step.

1. Implement the `PredictState` function.
This was not too difficult. The aim was to integrate the velocity and acceleration to get the predicted position and velocity. The `Rotate_BtoI` function made life a lot easier for converting from the body to the inertial frame. 

```c++
	//Integrate position
	predictedState(0) += predictedState[3] * dt;
	predictedState(1) += predictedState[4]* dt;
	predictedState(2) += predictedState[5] * dt; 
	
	//Integrate velocity in the Inertial Frame
	V3F intertialAccel = attitude.Rotate_BtoI(accel);
	predictedState(3) += intertialAccel.x * dt;
	predictedState(4) += intertialAccel.y * dt;
	predictedState(5) += ((float)intertialAccel.z - (float) CONST_GRAVITY) * dt;
```

This produced the desired output:
<p align="center">
   <img src="images/predict_state_g.jpg" width="300"/>
</p>

2. Covariance prediction
There are two parts to this implemenation. First we had to setup Rbg Prime based on the following matrix:

<p align="center">
   <img src="images/rbg_prime.jpg" width="600"/>
</p>

The code to implement this was pretty straight forward but it was worth triple checking as I did make an error with one of the entries. Luckily it was corrected before causing any problems.

```C++
	float sinPhi = sin(roll);
	float cosPhi = cos(roll);

	float sinTheta = sin(pitch);
	float cosTheta = cos(pitch);
	
	float sinPsi = sin(yaw);
	float cosPsi = cos(yaw);

	RbgPrime(0, 0) = -cosTheta * sinPsi;
	RbgPrime(0, 1) = -sinPhi * sinTheta * sinPsi - cosPhi * cosPsi;
	RbgPrime(0, 2) = -cosPhi * sinTheta * sinPsi + sinPhi * cosPsi;

	RbgPrime(1, 0) = cosTheta * cosPsi;
	RbgPrime(1, 1) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
	RbgPrime(1, 2) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
```

Next it was time to implement the `Predict` function to predict the current covariance forward by dt using the current accelerations and body rates as input. This required implementing the jacobian (gPrime) and then plugging in all of the variables into the covariance formula.

`gPrime` matrix:
<p align="center">
   <img src="images/jacobian.jpg" width="400"/>
</p>

My code solution as described above:<br/></br>

```C++
//add accelerations to a vector to enable matrix operations
	VectorXf accelVector(3);
	accelVector(0) = accel.x;
	accelVector(1) = accel.y;
	accelVector(2) = accel.z - (float)CONST_GRAVITY;

	gPrime(0, 3) = dt;
	gPrime(1, 4) = dt;
	gPrime(2, 5) = dt;
	gPrime(3, 6) = RbgPrime.row(0).dot(accelVector) * dt;
	gPrime(4, 6) = RbgPrime.row(1).dot(accelVector) * dt;
	gPrime(5, 6) = RbgPrime.row(2).dot(accelVector) * dt;

	//Equation to update the covariance
	ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```
<br/> **The resulting graph:**
<p align="center">
   <img src="images/covariance.gif" width="300"/>
</p>

In this next step you will be implementing the prediction step of your filter.


1. Run scenario `08_PredictState`.  This scenario is configured to use a perfect IMU (only an IMU). Due to the sensitivity of double-integration to attitude errors, we've made the accelerometer update very insignificant (`QuadEstimatorEKF.attitudeTau = 100`).  The plots on this simulation show element of your estimated state and that of the true state.  At the moment you should see that your estimated state does not follow the true state.

2. In `QuadEstimatorEKF.cpp`, implement the state prediction step in the `PredictState()` functon. If you do it correctly, when you run scenario `08_PredictState` you should see the estimator state track the actual state, with only reasonably slow drift, as shown in the figure below:

![predict drift](images/predict-slow-drift.png)

3. Now let's introduce a realistic IMU, one with noise.  Run scenario `09_PredictionCov`. You will see a small fleet of quadcopter all using your prediction code to integrate forward. You will see two plots:
   - The top graph shows 10 (prediction-only) position X estimates
   - The bottom graph shows 10 (prediction-only) velocity estimates
You will notice however that the estimated covariance (white bounds) currently do not capture the growing errors.

4. In `QuadEstimatorEKF.cpp`, calculate the partial derivative of the body-to-global rotation matrix in the function `GetRbgPrime()`.  Once you have that function implement, implement the rest of the prediction step (predict the state covariance forward) in `Predict()`.

**Hint: see section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the the transition model and the partial derivatives you may need**

**Hint: When it comes to writing the function for GetRbgPrime, make sure to triple check you've set all the correct parts of the matrix.**

**Hint: recall that the control input is the acceleration!**

5. Run your covariance prediction and tune the `QPosXYStd` and the `QVelXYStd` process parameters in `QuadEstimatorEKF.txt` to try to capture the magnitude of the error you see. Note that as error grows our simplified model will not capture the real error dynamics (for example, specifically, coming from attitude errors), therefore  try to make it look reasonable only for a relatively short prediction period (the scenario is set for one second).  A good solution looks as follows:

![good covariance](images/predict-good-cov.png)

Looking at this result, you can see that in the first part of the plot, our covariance (the white line) grows very much like the data.

If we look at an example with a `QPosXYStd` that is much too high (shown below), we can see that the covariance no longer grows in the same way as the data.

![bad x covariance](images/bad-x-sigma.PNG)

Another set of bad examples is shown below for having a `QVelXYStd` too large (first) and too small (second).  As you can see, once again, our covariances in these cases no longer model the data well.

![bad vx cov large](images/bad-vx-sigma.PNG)

![bad vx cov small](images/bad-vx-sigma-low.PNG)

***Success criteria:*** *This step doesn't have any specific measurable criteria being checked.*


### Step 4: Magnetometer Update ###

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, you will be adding the information from the magnetometer to improve your filter's performance in estimating the vehicle's heading.

1. Run scenario `10_MagUpdate`.  This scenario uses a realistic IMU, but the magnetometer update hasn’t been implemented yet. As a result, you will notice that the estimate yaw is drifting away from the real value (and the estimated standard deviation is also increasing).  Note that in this case the plot is showing you the estimated yaw error (`quad.est.e.yaw`), which is drifting away from zero as the simulation runs.  You should also see the estimated standard deviation of that state (white boundary) is also increasing.

2. Tune the parameter `QYawStd` (`QuadEstimatorEKF.txt`) for the QuadEstimatorEKF so that it approximately captures the magnitude of the drift, as demonstrated here:

![mag drift](images/mag-drift.png)

3. Implement magnetometer update in the function `UpdateFromMag()`.  Once completed, you should see a resulting plot similar to this one:

![mag good](images/mag-good-solution.png)

***Success criteria:*** *Your goal is to both have an estimated standard deviation that accurately captures the error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation.*

**Hint: after implementing the magnetometer update, you may have to once again tune the parameter `QYawStd` to better balance between the long term drift and short-time noise from the magnetometer.**

**Hint: see section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the magnetometer update.**


### Step 5: Closed Loop + GPS Update ###

1. Run scenario `11_GPSUpdate`.  At the moment this scenario is using both an ideal estimator and and ideal IMU.  Even with these ideal elements, watch the position and velocity errors (bottom right). As you see they are drifting away, since GPS update is not yet implemented.

2. Let's change to using your estimator by setting `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt`.  Rerun the scenario to get an idea of how well your estimator work with an ideal IMU.

3. Now repeat with realistic IMU by commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

4. Tune the process noise model in `QuadEstimatorEKF.txt` to try to approximately capture the error you see with the estimated uncertainty (standard deviation) of the filter.

5. Implement the EKF GPS Update in the function `UpdateFromGPS()`.

6. Now once again re-run the simulation.  Your objective is to complete the entire simulation cycle with estimated position error of < 1m (you’ll see a green box over the bottom graph if you succeed).  You may want to try experimenting with the GPS update parameters to try and get better performance.

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*

**Hint: see section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) for a refresher on the GPS update.**

At this point, congratulations on having a working estimator!

### Step 6: Adding Your Controller ###

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state.  So now, you will see how well your controller performs and de-tune your controller accordingly.

1. Replace `QuadController.cpp` with the controller you wrote in the last project.

2. Replace `QuadControlParams.txt` with the control parameters you came up with in the last project.

3. Run scenario `11_GPSUpdate`. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it.  Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

**Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.**

***Success criteria:*** *Your objective is to complete the entire simulation cycle with estimated position error of < 1m.*


## Tips and Tricks ##

 - When it comes to transposing matrices, `.transposeInPlace()` is the function you want to use to transpose a matrix

 - The [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj) document contains a helpful mathematical breakdown of the core elements on your estimator

## Submission ##

For this project, you will need to submit:

 - a completed estimator that meets the performance criteria for each of the steps by submitting:
   - `QuadEstimatorEKF.cpp`
   - `config/QuadEstimatorEKF.txt`

 - a re-tuned controller that, in conjunction with your tuned estimator, is capable of meeting the criteria laid out in Step 6 by submitting:
   - `QuadController.cpp`
   - `config/QuadControlParams.txt`

 - a write up addressing all the points of the rubric

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
