import numpy as np


def pwm_to_thrust(pwm_signal, k_T, k_omega, pwm_min):
    """
    Convert PWM signal to thrust using a quadratic model.

    Parameters:
    - pwm_signal (array or list): PWM values (typically in microseconds).
    - k_T (float): Thrust coefficient (determined experimentally).
    - k_omega (float): Speed gain coefficient.
    - pwm_min (float): Minimum PWM at which the motor starts spinning.

    Returns:
    - thrust (array): Estimated thrust values.
    """
    # Ensure PWM does not go below the minimum
    pwm_signal = np.maximum(pwm_signal, pwm_min)

    # Compute thrust using the quadratic model
    thrust = k_T * k_omega ** 2 * (pwm_signal - pwm_min) ** 2

    return thrust

def compute_angular_accel_error(gyro_data, thrust_data, dt, I_y, l, h):
    """
    Compute the error between actual and predicted angular acceleration.

    Parameters:
    - gyro_data (list or np.array): Gyroscope readings of pitch rate (ω_y) in rad/s.
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
    actual_accel = np.diff(gyro_data) / dt  # d(ω_y)/dt
    actual_accel = np.append(actual_accel, actual_accel[-1])  # Repeat last value to match length

    # Compute predicted angular acceleration using the model
    predicted_accel = []
    for i in range(len(thrust_data)):
        T1, T2, T3, T4 = thrust_data[i]
        theta = gyro_data[i] * dt  # Approximate small angle assumption

        # Model equation: ddot(theta) = (l * (T1 - T3) * cos(theta) + h * (T1 + T2 + T3 + T4) * sin(theta)) / I_y
        predicted_ddot_theta = (l * (T1 - T3) * np.cos(theta) + h * (T1 + T2 + T3 + T4) * np.sin(theta)) / I_y
        predicted_accel.append(predicted_ddot_theta)

    predicted_accel = np.array(predicted_accel)

    # Compute error between actual and predicted values
    error = actual_accel - predicted_accel

    return actual_accel, predicted_accel, error


# Example usage:
gyro_readings = [0.1, 0.12, 0.15, 0.17, 0.2]  # Example gyro pitch rates in rad/s
thrust_values = [(1.0, 1.2, 0.9, 1.1), (1.1, 1.3, 0.95, 1.2), (1.2, 1.4, 1.0, 1.3), (1.3, 1.5, 1.1, 1.4),
                 (1.4, 1.6, 1.2, 1.5)]  # Example thrust per motor
dt = 0.02  # 20ms timestep
I_y = 0.02  # Example moment of inertia
l = 0.1  # 10 cm arm length
h = 0.05  # 5 cm center of mass height

actual_accel, predicted_accel, error = compute_angular_accel_error(gyro_readings, thrust_values, dt, I_y, l, h)

# Print the results
print("Actual Angular Acceleration:", actual_accel)
print("Predicted Angular Acceleration:", predicted_accel)
print("Error:", error)
