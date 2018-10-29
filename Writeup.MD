
# Building an Estimator

### A. Writeup

Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it!

---
### B. Implement Estimator

**1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.**

*For the controls project, the simulator was working with a perfect set of sensors, meaning none of the sensors had any noise. The first step to adding additional realism to the problem, and developing an estimator, is adding noise to the quad's sensors. For the first step, you will collect some simulated noisy sensor data and estimate the standard deviation of the quad's sensor.

 - Run the simulator in the same way as you have before and choose scenario 06_NoisySensors. In this simulation, the interest is to record some sensor data on a static quad, so you will not see the quad move. You will see two plots at the bottom, one for GPS X position and one for The accelerometer's x measurement. The dashed lines are a visualization of a single standard deviation from 0 for each signal. The standard deviations are initially set to arbitrary values (after processing the data in the next step, you will be adjusting these values). If they were set correctly, we should see ~68% of the measurement points fall into the +/- 1 sigma bound. When you run this scenario, the graphs you see will be recorded to the following csv files with headers: config/log/Graph1.txt (GPS X data) and config/log/Graph2.txt (Accelerometer X data).

 - Process the logged files config/log/Graph1.txt (GPS X data) and config/log/Graph2.txt (Accelerometer X data) to figure out the standard deviation of the the GPS X signal and the IMU Accelerometer X signal.

![GPS Measurements](./images/GPS%20Measurements.png)  
 
![Accelerometer Measurements](./images/Accelerometer%20Measurements.png)  

![Jupyter Notebook Used to Generate Statistics](./GPS_Accelerometer_Measurements.ipynb)

 - Plug in your result into the top of config/6_Sensornoise.txt. Specially, set the values for MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY to be the values you have calculated.

 - Run the simulator. If your values are correct, the dashed lines in the simulation will eventually turn green, indicating you’re capturing approx 68% of the respective measurements (which is what we expect within +/- 1 sigma bound for a Gaussian noise model)

![06_SensorNoise](./images/06_SensorNoise.gif)  

Success criteria: Your standard deviations should accurately capture the value of approximately 68% of the respective measurements.

NOTE: Your answer should match the settings in SimulatedSensors.txt, where you can also grab the simulated noise parameters for all the other sensors.

*The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.*  

The GPS sensor capture data with frequency 0.05 and accelerometer captures data at rate 0.1, so there are 20x more accelerometer points than GPS points.  From the lectures accelerometer data is more frequent but more uncertain.  GPS is less frequent but more reliable than acceleromoter.

I changed the sensor simulation to capture over 1600 GPS points and 333000 Accelerometer points


From the statistics calculated I used GPS Std = 0.5 and acceleromter std = 0.7 rounding.  These numbers in fact matched the sensors text mentioned in the readme.

Plugging in the stds and running the 06sensors scenario the tests pass.  Here is a clip of the test.


**2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.****

*The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.*  

**3. Implement all of the elements of the prediction step for the estimator.**

*The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.*  

**4. Implement the magnetometer update.**

*The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).*  

**5. Implement the GPS update.**

*The estimator should correctly incorporate the GPS information to update the current state estimate.*  

--- 
### C. Flight Evaluation

*For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.*  

&De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.*  

