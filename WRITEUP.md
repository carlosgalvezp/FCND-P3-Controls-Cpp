## Project: Controls
[![Submission Video](http://img.youtube.com/vi/DZ2NLhmdjdU/0.jpg)](https://youtu.be/DZ2NLhmdjdU)

---


## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Implemented Controller

#### 1. Implemented body rate control in python and C++.

```python
class PIDController(object):
    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.error_sum = 0.0

    def control(self, error, error_dot, feed_forward=0.0):
        self.error_sum += error
        return self.k_p * error + self.k_i * self.error_sum + self.k_d * error_dot + feed_forward

class PDController(PIDController):
    def __init__(self, k_p, k_d):
        super().__init__(k_p=k_p, k_i=0.0, k_d=k_d)

    def control(self, error, error_dot, feed_forward=0.0):
        return super().control(error, error_dot, feed_forward)

class PController(PDController):
    def __init__(self, k_p):
        super().__init__(k_p=k_p, k_d=0.0)

    def control(self, error):
        return super().control(error, error_dot=0.0, feed_forward=0.0)
```

```python
def body_rate_control(self, body_rate_cmd, body_rate):
    """ Generate the roll, pitch, yaw moment commands in the body frame

    Args:
        body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
        body_rate: 3-element numpy array (p,q,r) in radians/second^2

    Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
    """
    moment_p = MOI[0] * self.p_controller_.control(body_rate_cmd[0] - body_rate[0])
    moment_q = MOI[1] * self.q_controller_.control(body_rate_cmd[1] - body_rate[1])
    moment_r = MOI[2] * self.r_controller_.control(body_rate_cmd[2] - body_rate[2])

    moment_p = np.clip(moment_p, -MAX_TORQUE, MAX_TORQUE)
    moment_q = np.clip(moment_q, -MAX_TORQUE, MAX_TORQUE)
    moment_r = np.clip(moment_r, -MAX_TORQUE, MAX_TORQUE)

    return np.array([moment_p, moment_q, moment_r])
```

```cpp
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
    //   pqrCmd: desired body rates [rad/s]
    //   pqr: current or estimated body rates [rad/s]
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes

    // HINTS:
    //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
    //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
    //  - you'll also need the gain parameter kpPQR (it's a V3F)

    V3F momentCmd;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    const V3F pqr_dot = kpPQR * (pqrCmd - pqr);
    momentCmd = V3F(Ixx, Iyy, Izz) * pqr_dot;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return momentCmd;
}
```

#### 2. Implement roll pitch control in python and C++.

```python
def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
    """ Generate the rollrate and pitchrate commands in the body frame

    Args:
        target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
        attitude: 3-element numpy array (roll, pitch, yaw) in radians
        thrust_cmd: vehicle thrust command in Newton

    Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
    """
    roll_pitch_rate_cmd = np.array([0.0, 0.0])

    if abs(thrust_cmd) > EPSILON:
        R = euler2RM(*attitude)

        if abs(R[2][2]) > EPSILON:
            # Thrust comes with positive up, but in NED it should be positive down!
            # Also, b_* must be dimensionless so convert thrust to acceleration
            b_c_x = acceleration_cmd[0] / (-thrust_cmd / DRONE_MASS_KG)
            b_c_y = acceleration_cmd[1] / (-thrust_cmd / DRONE_MASS_KG)

            b_a_x = R[0,2]
            b_a_y = R[1,2]

            b_c_x_dot = self.roll_controller_.control(b_c_x - b_a_x)
            b_c_y_dot = self.pitch_controller_.control(b_c_y - b_a_y)

            M = np.array([[R[1,0], -R[0,0]],
                            [R[1,1], -R[0,1]]])
            b_c_dot = np.array([b_c_x_dot, b_c_y_dot])

            roll_pitch_rate_cmd = (1.0 / R[2,2]) * np.matmul(M, b_c_dot)
        else:
            print('R[2][2] = 0.0, cannot compute roll_pitch_rate!')
    else:
        print('thrust_cmd = 0.0, cannot compute roll_pitch_rate!')

    return roll_pitch_rate_cmd
```

```cpp
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
    // Calculate a desired pitch and roll angle rates based on a desired global
    //   lateral acceleration, the current attitude of the quad, and desired
    //   collective thrust command
    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the roll/pitch gain kpBank
    //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Current attitude
    const float b_x_a = R(0,2);
    const float b_y_a = R(1,2);

    // Target attitude
    const float thrust_acceleration = -collThrustCmd / mass;
    const float b_x_c = accelCmd.x / (thrust_acceleration);
    const float b_y_c = accelCmd.y / (thrust_acceleration);

    // Commanded rates in world frame
    const float b_x_c_dot = kpBank * (b_x_c - b_x_a);
    const float b_y_c_dot = kpBank * (b_y_c - b_y_a);

    // Roll and pitch rates
    const float r_33_inv = 1.0F / R(2,2);
    pqrCmd.x =  r_33_inv * (R(1,0)*b_x_c_dot - R(0,0)*b_y_c_dot);
    pqrCmd.y =  r_33_inv * (R(1,1)*b_x_c_dot - R(0,1)*b_y_c_dot);
    pqrCmd.z = 0.0F;  // yaw controller set in YawControl
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return pqrCmd;
}

```

#### 3. Implement altitude control in python.

```python
def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
    """Generate vertical acceleration (thrust) command

    Args:
        altitude_cmd: desired vertical position (+up)
        vertical_velocity_cmd: desired vertical velocity (+up)
        altitude: vehicle vertical position (+up)
        vertical_velocity: vehicle vertical velocity (+up)
        attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
        acceleration_ff: feedforward acceleration command (+up)

    Returns: thrust command for the vehicle (+up)
    """
    thrust = 0.0

    R = euler2RM(*attitude)
    b_z = R[2][2]

    if abs(b_z) > EPSILON:
        error_z = altitude_cmd - altitude
        error_z_dot = vertical_velocity_cmd - vertical_velocity

        u_1_bar = self.altitude_controller_.control(error_z, error_z_dot, acceleration_ff)
        thrust = DRONE_MASS_KG * ((u_1_bar - (-GRAVITY)) / b_z)
        thrust = np.clip(thrust, 0.0, MAX_THRUST)
    else:
        print('b_z = 0.0, cannot compute thrust!')

    return thrust
```

#### 4. Implement altitude controller in C++.

```cpp
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ,
                                   Quaternion<float> attitude, float accelZCmd, float dt)
{
    // Calculate desired quad thrust based on altitude setpoint, actual altitude,
    //   vertical velocity setpoint, actual vertical velocity, and a vertical
    //   acceleration feed-forward command
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the gain parameters kpPosZ and kpVelZ
    //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
    //  - make sure to return a force, not an acceleration
    //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Get z component of the thrust
    const float b_z = R(2,2);

    // Constrain commanded velocity (NED, descending means higher Z)
    velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

    // Compute error
    const float error = posZCmd - posZ;
    const float error_dot = velZCmd - velZ;
    integratedAltitudeError += error * dt;

    // Compute desired acceleration
    const float u1_bar = kpPosZ * error + \
                         kpVelZ * error_dot + \
                         KiPosZ * integratedAltitudeError + \
                         accelZCmd;
    float acc_z_desired = (u1_bar - CONST_GRAVITY) / b_z;

    // Compute thrust (positive upwards)
    thrust = -acc_z_desired * mass;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return thrust;
}
```

#### 5. Implement lateral position control in python and C++.

```python
def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                                acceleration_ff = np.array([0.0, 0.0])):
    """Generate horizontal acceleration commands for the vehicle in the local frame

    Args:
        local_position_cmd: desired 2D position in local frame [north, east]
        local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
        local_position: vehicle position in the local frame [north, east]
        local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
        acceleration_cmd: feedforward acceleration command

    Returns: desired vehicle 2D acceleration in the local frame [north, east]
    """
    acc_x = self.x_controller_.control(local_position_cmd[0] - local_position[0],
                                        local_velocity_cmd[0] - local_velocity[0],
                                        acceleration_ff[0])

    acc_y = self.y_controller_.control(local_position_cmd[1] - local_position[1],
                                        local_velocity_cmd[1] - local_velocity[1],
                                        acceleration_ff[1])

    return np.array([acc_x, acc_y])
```

```cpp
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
    // Calculate a desired horizontal acceleration based on
    // desired lateral position/velocity/acceleration and current pose
    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmd: desired acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    // HINTS:
    //  - use fmodf(foo,b) to constrain float foo to range [0,b]
    //  - use the gain parameters kpPosXY and kpVelXY
    //  - make sure you cap the horizontal velocity and acceleration
    //    to maxSpeedXY and maxAccelXY

    // make sure we don't have any incoming z-component
    accelCmd.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Contrain desired velocity
    velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
    velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

    // Compute PD controller + feedforward
    const V3F error = posCmd - pos;
    const V3F error_dot = velCmd - vel;

    accelCmd = kpPosXY*error + kpVelXY*error_dot + accelCmd;

    // Constrain desired acceleration
    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
    accelCmd.z = 0.0F;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return accelCmd;
}

```

#### 6. Implement yaw control in python and C++.

```python
def yaw_control(self, yaw_cmd, yaw):
    """ Generate the target yawrate

    Args:
        yaw_cmd: desired vehicle yaw in radians
        yaw: vehicle yaw in radians

    Returns: target yawrate in radians/sec
    """
    error = normalize_angle(yaw_cmd - yaw)
    return self.yaw_controller_.control(error)
```

```cpp
float QuadControl::YawControl(float yawCmd, float yaw)
{
    // Calculate a desired yaw rate to control yaw to yawCmd
    // INPUTS:
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    // HINTS:
    //  - use fmodf(foo,b) to constrain float foo to range [0,b]
    //  - use the yaw control gain parameter kpYaw

    float yawRateCmd=0;
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    const float error = normalizeAngle(yawCmd - yaw);
    yawRateCmd = kpYaw * error;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return yawRateCmd;
}
```

#### 7. Implement calculating the motor commands given commanded thrust and moments in C++.

```cpp
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
    // Convert a desired 3-axis moment and collective thrust command to
    //     individual motor thrust commands
    // INPUTS:
    //     collThrustCmd: desired collective thrust [N]
    //     momentCmd: desired rotation moment about each axis [N m]
    // OUTPUT:
    //     set class member variable cmd (class variable for graphing) where
    //     cmd.desiredThrustsN[0..3]: motor commands, in [N]

    // HINTS:
    // - you can access parts of desMoment via e.g. desMoment.x
    // You'll need the arm length parameter L, and the drag/thrust ratio kappa

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    const float l = L * 0.5F * sqrt(2.0F);  // Arm length perpendicular to X-Y axis
    const float l_inv = 1.0F / l;
    const float k_inv = 1.0F / kappa;

    const float f = collThrustCmd;
    const float t_x = momentCmd.x;
    const float t_y = momentCmd.y;
    const float t_z = momentCmd.z;

    // This comes from the matrix equation:
    // [ 1  1  1  1][F1]     [Ft   ]
    // [ L -L  L -L][F2]     [tau_x]
    // [ L  L -L -L][F3] =   [tau_y]
    // [-K  K  K -K][F4]     [tau_z]

    // Differences w.r.t. Python version:
    // 1) The motors spin in opposite direction
    // 2) M3 and M4 are swapped

    // The inverse of the 4x4 matrix is:
    // [0.25,  0.25/L,  0.25/L, -0.25/K],
    // [0.25, -0.25/L,  0.25/L,  0.25/K],
    // [0.25,  0.25/L, -0.25/L,  0.25/K],
    // [0.25, -0.25/L, -0.25/L, -0.25/K]]
    cmd.desiredThrustsN[0] = 0.25 * (f + l_inv*t_x + l_inv*t_y - k_inv*t_z); // front left
    cmd.desiredThrustsN[1] = 0.25 * (f - l_inv*t_x + l_inv*t_y + k_inv*t_z); // front right
    cmd.desiredThrustsN[2] = 0.25 * (f + l_inv*t_x - l_inv*t_y + k_inv*t_z); // rear left
    cmd.desiredThrustsN[3] = 0.25 * (f - l_inv*t_x - l_inv*t_y - k_inv*t_z); // rear right

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return cmd;
}
```

### Flight Evaluation

#### 1. Your python controller is successfully able to fly the provided test trajectory, meeting the minimum flight performance metrics.

#### 2. Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
