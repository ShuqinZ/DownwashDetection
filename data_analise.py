import numpy as np

from downwash_sensing import load_data


def calculate_k_T(C_T, rho, propeller_diameter):
    """
    Calculate the thrust coefficient k_T based on the momentum theory.

    Parameters:
    - C_T (float): Thrust coefficient (depends on propeller shape).
    - rho (float): Air density (kg/m^3).
    - propeller_diameter (float): Diameter of the propeller in meters.

    Returns:
    - k_T (float): Thrust coefficient in N/(rad/s)^2.
    """
    R = propeller_diameter / 2  # Convert diameter to radius
    A = np.pi * R ** 2  # Disk area
    k_T = 0.5 * C_T * rho * A * R ** 2
    return k_T


def get_thrust(k_T, K_V, V):
    """
    Convert PWM signal to thrust using a quadratic model.

    Parameters:
    - pwm_signal (array or list): PWM values (typically in microseconds).
    - k_T (float): Thrust coefficient (determined experimentally).
    - k_V (float):
    - V: Real time voltage

    Returns:
    - thrust (array): Estimated thrust values.
    """

    if isinstance(V, np.ndarray):
        thrust = []
        for v in V:
            omega = K_V * v
            # Compute thrust using the quadratic model
            thrust.append(k_T * omega ** 2)
    else:
        omega = K_V * V

        # Compute thrust using the quadratic model
        thrust = k_T * omega ** 2

    return thrust


def compute_angular_accel_error(gyro_data, motor_voltage, dt, I_y, k_T, K_V, l, h):
    """
    Compute the error between actual and predicted angular acceleration.

    Parameters:
    - gyro_data (list or np.array): Gyroscope readings of roll and pitch (rad).
    - thrust_data (list or np.array): A list of tuples [(T1, T2, T3, T4), ...] with thrust values per time step.
    - dt (float): Time step between gyro readings in seconds.
    - I_y (float): Moment of inertia about the pitch axis.
    - l (float): Arm length from center to motor.
    - h (float): Center of mass height above thrust plane.

    Returns:
    - actual_accel (np.array): Numerically computed angular acceleration from gyro readings.
    - predicted_accel (np.array): Computed from the mathematical model.
    - error (np.array): Difference between actual and predicted acceleration.
    """

    # Compute actual angular acceleration (numerical derivative of gyro readings)
    gyro_rate = np.diff(gyro_data) / dt
    actual_accel = np.diff(gyro_rate) / dt  # d(ω)/dt
    actual_accel = np.append(actual_accel, actual_accel[-1])  # Repeat last value to match length

    # Compute predicted angular acceleration using the model
    predicted_accel = []
    for i in range(len(motor_voltage)):
        T1, T2, T3, T4 = get_thrust(k_T, K_V, motor_voltage)
        theta = gyro_rate[i] * dt  # Approximate small angle assumption

        # Model equation: ddot(theta) = (l * (T1 - T3) * cos(theta) + h * (T1 + T2 + T3 + T4) * sin(theta)) / I_y
        predicted_ddot_theta = (l * (T1 - T3) * np.cos(theta) + h * (T1 + T2 + T3 + T4) * np.sin(theta)) / I_y
        predicted_accel.append(predicted_ddot_theta)

    predicted_accel = np.array(predicted_accel)

    # Compute error between actual and predicted values
    error = actual_accel - predicted_accel

    return actual_accel, predicted_accel, error


if __name__ == '__main__':
    filepath = ""
    log_vars = load_data("metrics/" + filepath)


    gyro_data = [0.1, 0.12, 0.15, 0.17, 0.2]  # Example gyro readings in rad
    motor_voltage = [(1.0, 1.2, 0.9, 1.1), (1.1, 1.3, 0.95, 1.2), (1.2, 1.4, 1.0, 1.3), (1.3, 1.5, 1.1, 1.4),
                     (1.4, 1.6, 1.2, 1.5)]  # Example thrust per motor

    dt = 0.01  # 10ms timestep
    I_y = 0.000023951  # moment of inertia, kg/m^2
    l = (0.092/2) * np.sqrt(2)  # arm length
    h = -0.01  # -1 cm center of mass height
    K_V = 13  # rpm/mV
    C_T = 0.1  # depend on Propeller Shape
    rho = 1.225  # air pressure
    propeller_diameter = 0.051  # meter

    k_T = calculate_k_T(C_T, rho, propeller_diameter)

    actual_accel, predicted_accel, error = compute_angular_accel_error(gyro_data, motor_voltage, dt, I_y, k_T, K_V, l, h)

    # Print the results
    print("Actual Angular Acceleration:", actual_accel)
    print("Predicted Angular Acceleration:", predicted_accel)
    print("Error:", error)
