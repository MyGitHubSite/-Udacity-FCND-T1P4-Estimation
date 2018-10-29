
# Building an Estimator (WORK IN PROGRESS)

### A. Writeup

Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it!

---
### B. Implement Estimator

**Step 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.**

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


**Step 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.****

*The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.*  

Currently a small-angle approximation integration method is implemented.  The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers.
  
**Old Code:**  

    float predictedPitch = pitchEst + dtIMU * gyro.y;
    float predictedRoll = rollEst + dtIMU * gyro.x;
    ekfState(6) = ekfState(6) + dtIMU * gyro.z;	// yaw
    
    //normalize yaw to -pi .. pi
    if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
    if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))

**New Code:**  

    Quaternion<float> quaternion;     
    Quaternion<float> quat = quaternion.FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
    Quaternion<float> quatEst = quaternion.IntegrateBodyRate(gyro, dtIMU) * quat;   

    float predictedPitch = quatEst.Pitch();     // Get pitch from quaterion
    float predictedRoll = quatEst.Roll();       // Get roll from quaterion
    ekfState(6) = fmod(quatEst.Yaw(), F_PI);    // Get yaw from quaterion and normalize -pi to pi

![07_AttitudeEstimation](./images/07_AttitudeEstimation.gif)  

**Step 3. Implement all of the elements of the prediction step for the estimator.**

*The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.*  

**state update element (PredictState() function)**  

    V3F attitudeI = attitude.Rotate_BtoI(accel);                  //rotate vector from body to inertial frame
    // Position
    predictedState(0) = curState(0) + curState(3) * dt;           // x
    predictedState(1) = curState(1) + curState(4) * dt;           // y
    predictedState(2) = curState(2) + curState(5) * dt;           // z
    // Velocity
    predictedState(3) = curState(3) + attitudeI.x * dt;           // x_dot
    predictedState(4) = curState(4) + attitudeI.y * dt;           // y_dot
    predictedState(5) = curState(5) + (attitudeI.z - 9.81f) * dt; // z_dot
    // Yaw
    predictedState(6) = curState(6);                              // psi

**Calculation of the Rgb Prime Matrix:**  

    float phi = pitch;
    float theta = roll;
    float psi = yaw;

    RbgPrime(0, 0) = -cos(theta)*sin(psi);
    RbgPrime(1, 0) =  cos(theta)*cos(psi);
    RbgPrime(2, 0) = 0;

    RbgPrime(0, 1) = -sin(phi)*sin(theta)*sin(psi) - cos(phi)*cos(psi);
    RbgPrime(1, 1) =  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
    RbgPrime(2, 1) = 0;

    RbgPrime(0, 2) = -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi);
    RbgPrime(1, 2) =  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    RbgPrime(2, 2) = 0;

**Update of the State Covariance**  


**Step 4. Implement the magnetometer update.**

*The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).*  

**Step 5. Implement the GPS update.**

*The estimator should correctly incorporate the GPS information to update the current state estimate.*  

--- 
### C. Flight Evaluation

*For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.*  

&De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.*  

