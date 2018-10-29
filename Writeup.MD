
# Building an Estimator

### A. Writeup

Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it!

---
### B. Implement Estimator

**1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.**

The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

The GPS sensor capture data with frequency 0.05 and accelerometer captures data at rate 0.1, so there are 20x more accelerometer points than GPS points.  From the lectures accelerometer data is more frequent but more uncertain.  GPS is less frequent but more reliable than acceleromoter.

I changed the sensor simulation to capture over 1600 GPS points and 333000 Accelerometer points

![GPS Measurements](./images/GPS%20Measurements.png)  
 
![Accelerometer Measurements](./images/Accelerometer%20Measurements.png)  

From the statistics calculated I used GPS Std = 0.5 and acceleromter std = 0.7 rounding.  These numbers in fact matched the sensors text mentioned in the readme.

Plugging in the stds and running the 06sensors scenario the tests pass.  Here is a clip of the test.

![06_SensorNoise](./images/06_SensorNoise.gif)  



**2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.****

The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.

**3. Implement all of the elements of the prediction step for the estimator.**

The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.

**4. Implement the magnetometer update.**

The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

**5. Implement the GPS update.**

The estimator should correctly incorporate the GPS information to update the current state estimate.

--- 
### C. Flight Evaluation

For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.

De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

