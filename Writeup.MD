

# Building an Estimator (WORK IN PROGRESS)






In this project, the estimation portion of the controller used in the CPP simulator needs to be developed. By the end of the project, the simulated quadrotor will be flying with the estimator and the custom controller from the previous project.

**The Tasks:**  

  Step 1: Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.  
  Step 2: Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.  
  Step 3: Implement all of the elements of the prediction step for the estimator.  
  Step 4: Implement the magnetometer update.  
  Step 5: Implement the GPS update.  
  Step 6: De-tune the controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.  
  
---
#### Step 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
---

For better estimation of the standard deviations I changed the sensor simulation to capture over 1,600 GPS points and 32,000 Accelerometer points.

*Process the logged files config/log/Graph1.txt (GPS X data) and config/log/Graph2.txt (Accelerometer X data) to figure out the standard deviation of the the GPS X signal and the IMU Accelerometer X signal.*  

The calculated GPS Standard Deviation was 0.706 and acceleromter standard deviation was 0.501.  

![GPS Measurements](./images/GPS%20Measurements.png)  
 
![Accelerometer Measurements](./images/Accelerometer%20Measurements.png)  
![Jupyter Notebook Used to Generate Statistics](./GPS_Accelerometer_Measurements.ipynb)

 *Plug in your result into the top of config/6_Sensornoise.txt. Specially, set the values for MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY to be the values you have calculated.*  
 
The standard deviations were updated in config/6_Sensornoise.txt.  

*Run the simulator. If your values are correct, the dashed lines in the simulation will eventually turn green, indicating you’re capturing approx 68% of the respective measurements (which is what we expect within +/- 1 sigma bound for a Gaussian noise model)*  

The simulutor passed the Sensor Noise test.

![06_Sensor Noise Simulation Results](./images/06_SensorNoise_Sim.PNG)  

![06_SensorNoise](./images/06_SensorNoise.gif)  

---
#### Step 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.  
---
*The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.*  

Currently a small-angle approximation integration method is implemented.  The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers.

**Old Code:**  

    float predictedPitch = pitchEst + dtIMU * gyro.y;
    float predictedRoll = rollEst + dtIMU * gyro.x;
    ekfState(6) = ekfState(6) + dtIMU * gyro.z;	// yaw. 

![07_Attitude Error(Linear Estimation)](./images/Attitude_Error_Linear_Estimation.PNG)  

Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))

**New Code:**  

![qt_bar](./images/qt_bar.PNG)  
  
    Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
    Quaternion<float> qt_bar = qt;
    qt_bar.IntegrateBodyRate(gyro, dtIMU);   

![Predicted_Pitch_Roll](./images/Predicted_Pitch_Roll.PNG)  

    float predictedRoll = qt_bar.Roll();       // Get roll from quaterion
    float predictedPitch = qt_bar.Pitch();     // Get pitch from quaterion
    ekfState(6) = qt_bar.Yaw();                // Get yaw from quaterion  

![07_Attitude Error(Non-Linear Estimation)](./images/Attitude_Error_Non-Linear_Estimation.PNG)  

![07_Attitude Estimation Simulation Results](./images/07_Attitude_Estimation_Sim.PNG)  

![07_AttitudeEstimation](./images/07_AttitudeEstimation.gif)  
---
#### Step 3. Implement all of the elements of the prediction step for the estimator.  
---
*The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.*  

**State Update Element (PredictState() Function)**  
  
  // Predict the current state forward by time dt using current accelerations and body rates as input
  // INPUTS: 
  //   curState: starting state
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  
    V3F accel_i = attitude.Rotate_BtoI(accel);                  //rotate vector from body to inertial frame

    predictedState(0) = curState(0) + curState(3) * dt;           // x_bar
    predictedState(1) = curState(1) + curState(4) * dt;           // y_bar
    predictedState(2) = curState(2) + curState(5) * dt;           // z_bar
    predictedState(3) = curState(3) + accel_i.x * dt;             // x_dot_bar
    predictedState(4) = curState(4) + accel_i.y * dt;             // y_dot_bar
    predictedState(5) = curState(5) + (accel_i.z - 9.81f) * dt;   // z_dot_bar

![Predict State Before](./images/Predict_State_Before.PNG)
![Predict State After](./images/Predict_State_After.PNG)

![Predict_State](./images/Predict_State_Before.gif)  

![]{./images/08_Predicted_State.GIF)

**Calculation of the Rgb Prime Matrix:**  

![Rgb Prime Matrix](./images/RgbPrime.PNG)  

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

![g Prime Matrix](./images/gPrime_Jacobian.PNG)  

    gPrime(0, 3) = dt;
    gPrime(1, 4) = dt;
    gPrime(2, 5) = dt;

    gPrime(3, 6) = (RbgPrime(0, 0) * accel.x * RbgPrime(0, 1) * accel.y * RbgPrime(0, 2) * accel.z) * dt;
    gPrime(4, 6) = (RbgPrime(1, 0) * accel.x * RbgPrime(1, 1) * accel.y * RbgPrime(1, 2) * accel.z) * dt;
    gPrime(5, 6) = (RbgPrime(2, 0) * accel.x * RbgPrime(2, 1) * accel.y * RbgPrime(2, 2) * accel.z) * dt;
    
![Extended Kalman Filter Predit](./images/EKF_Predict.PNG)  

    MatrixXf gPrimeT = gPrime;
    gPrimeT.transposeInPlace();
    ekfCov = gPrime * ekfCov * gPrimeT + Q;

Paramter Update:

    QPosXYStd = .03  # 0.5

![Predict Covariance Before](./images/Predict_Covariance_Before.PNG)
![Predict Covariance After](./images/Predict_Covariance_After.PNG)
![Predict_Covariance](./images/Predict_Covariance.gif)  

---
#### Step 4. Implement the Magnetometer Update.  
---
*The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).*  


![ZfromX](./images/Mag_ZfromX.PNG)  
![hPrime](./images/Mag_hPrime.PNG)  

Code:  

    zFromX(0) = ekfState(6);
    hPrime(0, 6) = 1;

    // Normalize the Yaw Difference
    // Error should be the short way around, not the long way.

    float yawDiff = magYaw - ekfState(6);

    if (yawDiff > F_PI) {
	   zFromX(0) += 2.f*F_PI;
    }
    else if (yawDiff < -F_PI) {
	   zFromX(0) -= 2.f*F_PI;
    }

Parameter Update:

    QYawStd = .08   # .05


![]{./images/Mag_Yaw_Error_Before.PNG)
![]{./images/Mag_Yaw_error_After.PNG)
![]{./images/Mag_Update_Sim.PNG)
![]{./images/10_MagUpdate.GIF)





---  
#### Step 5. Implement the GPS update.  
---
*The estimator should correctly incorporate the GPS information to update the current state estimate.*  

![ZfromX](./images/GPS_ZfromX.PNG)  
![hPrime](./images/GPS_hPrime.PNG)  

Code:  

    zFromX(0) = ekfState(0);
    zFromX(1) = ekfState(1);
    zFromX(2) = ekfState(2);
    zFromX(3) = ekfState(3);
    zFromX(4) = ekfState(4);
    zFromX(5) = ekfState(5);
  
    hPrime(0, 0) = 1;
    hPrime(1, 1) = 1;
    hPrime(2, 2) = 1;
    hPrime(3, 3) = 1;
    hPrime(4, 4) = 1;
    hPrime(5, 5) = 1;

![]{./images/GPS_Update_Before.PNG)
![]{./images/GPS_Update_After.PNG)
![]{./images/11_GPS_Update_Sim.PNG)
![]{./images/11_GPS_Update.GIF)


--- 
### C. Flight Evaluation

*For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.*  

---
#### Step 6: Adding Your Controller
---
*De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.*  

Up to this point, we have been working with a controller that has been relaxed to work with an estimated state instead of a real state. So now, you will see how well your controller performs and de-tune your controller accordingly.

Replace QuadController.cpp with the controller you wrote in the last project.

Replace QuadControlParams.txt with the control parameters you came up with in the last project.

Run scenario 11_GPSUpdate. If your controller crashes immediately do not panic. Flying from an estimated state (even with ideal sensors) is very different from flying with ideal pose. You may need to de-tune your controller. Decrease the position and velocity gains (we’ve seen about 30% detuning being effective) to stabilize it. Your goal is to once again complete the entire simulation cycle with an estimated position error of < 1m.

Hint: you may find it easiest to do your de-tuning as a 2 step process by reverting to ideal sensors and de-tuning under those conditions first.

Success criteria: Your objective is to complete the entire simulation cycle with estimated position error of < 1m.

**De-tuned Parameters:**  

    # Position control gains
    kpPosXY = 30        # 40
    kpPosZ = 20         # 55
    KiPosZ = 50         # 50

    # Velocity control gains
    kpVelXY = 13        # 15
    kpVelZ = 9          # 15

    # Angle control gains
    kpBank = 10         # 12    
    kpYaw = 2           # 4     

    # Angle rate gains
    kpPQR = 85, 85, 6   # 40, 40, 10  


# Process noise model
# note that the process covariance matrix is diag(pow(QStd,2))*dtIMU

QPosXYStd = .03 # .05
QPosZStd = .05  # .05
QVelXYStd = .1  # .05
QVelZStd = .1   # .1
QYawStd = .08   # .05

# GPS measurement std deviations
GPSPosXYStd = 1
GPSPosZStd = 100 #3
GPSVelXYStd = .1
GPSVelZStd = .3

# Magnetometer
MagYawStd = .1

dtIMU = 0.002
attitudeTau = 100

