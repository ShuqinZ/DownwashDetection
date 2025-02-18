from util.data_process import *


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


def compute_angular_accel_error(time_line, gyro_data, motor_voltage, I_xyz, k_T, K_V, l, h):
    """
    Compute the error between actual and predicted angular acceleration for roll, pitch, and yaw.

    Parameters:
    - time_line (list): Time stamp in sec
    - gyro_data (dict): Gyroscope readings { 'roll': np.array, 'pitch': np.array, 'yaw': np.array } in rad.
    - motor_voltage (list or np.array): A list of tuples [(V1, V2, V3, V4), ...] with motor voltages per time step.
    - I_x (float): Moment of inertia about the roll axis.
    - I_y (float): Moment of inertia about the pitch axis.
    - I_z (float): Moment of inertia about the yaw axis.
    - l (float): Arm length from center to motor.
    - h (float): Center of mass height above thrust plane.

    Returns:
    - actual_accel (dict): Numerically computed angular acceleration for roll, pitch, and yaw.
    - predicted_accel (dict): Computed from the mathematical model.
    - error (dict): Difference between actual and predicted acceleration.
    """

    actual_accel = [[] for _ in range(3)]
    predicted_accel = [[] for _ in range(3)]

    I_x, I_y, I_z = I_xyz

    for axis in range(3):
        # Compute actual angular acceleration (numerical derivative of gyro readings)
        gyro_rate = np.diff(gyro_data[:, axis]) / np.diff(time_line)  # Angular velocity
        actual_accel[axis] = np.diff(gyro_rate.transpose()) / np.diff(time_line[1:])   # Angular acceleration

        # Adjust length to match time steps
        # actual_accel[axis] = np.append(actual_accel[axis], actual_accel[axis][-1])  # Repeat last value

    time_line = time_line[1:]

    for i in range(len(actual_accel[0])):
        dt = (time_line[i + 1] - time_line[i])

        # Compute thrust for all four motors
        T1, T2, T3, T4 = get_thrust(k_T, K_V, motor_voltage[i])

        # Compute small-angle approximations
        theta = gyro_data[i, 0] * dt
        phi = gyro_data[i, 1] * dt
        # psi = gyro_data[2][i] * dt

        # Roll equation: ddot(phi) = (l * (T2 - T4) * cos(phi)) / I_x
        predicted_ddot_roll = (l * (T2 - T4) * np.cos(phi)) / I_x

        # Pitch equation: ddot(theta) = (l * (T1 - T3) * cos(theta) + h * (T1 + T2 + T3 + T4) * sin(theta)) / I_y
        predicted_ddot_pitch = (l * (T1 - T3) * np.cos(theta) + h * (T1 + T2 + T3 + T4) * np.sin(theta)) / I_y

        # Yaw equation: ddot(psi) = (torque from motor differentials) / I_z
        predicted_ddot_yaw = (l * ((T1 + T3) - (T2 + T4))) / I_z

        # Store computed values
        predicted_accel[0].append(predicted_ddot_roll)
        predicted_accel[1].append(predicted_ddot_pitch)
        predicted_accel[2].append(predicted_ddot_yaw)

    # Convert lists to numpy arrays
    for axis in range(3):
        predicted_accel[axis] = np.array(predicted_accel[axis])

    # Compute errors
    residue = [np.array(actual_accel[axis]) - np.array(predicted_accel[axis]) for axis in range(3)]

    return actual_accel, predicted_accel, residue


if __name__ == '__main__':

    configs = ["x0_z8_yaw0_TH35000"]

    project_root = Path(__file__).resolve().parent

    dir_path = os.path.join(project_root, "metrics")

    for config in configs:
        directory_name = os.path.join(dir_path, config)
        file_names = list_files(directory_name)

        for f in file_names:
            if f.split('.')[-1] != "json":
                continue

            filepath = os.path.join(directory_name, f)
            log_vars = load_data(filepath)

            timeline_0 = np.array(log_vars["timestamp"])
            gyro_data = [
                [roll, pitch, yaw]
                for roll, pitch, yaw in zip(
                    log_vars["component"]["Gyro"]["value"]["stateEstimate.roll"]["data"],
                    log_vars["component"]["Gyro"]["value"]["stateEstimate.pitch"]["data"],
                    log_vars["component"]["Gyro"]["value"]["stateEstimate.yaw"]["data"]
                )
            ]

            # timeline_1 = log_vars["Gyro_Accel"]["timestamp"]
            motor_PWM = np.array([
                [m1, m2, m3, m4]
                for m1, m2, m3, m4 in zip(
                    log_vars["component"]["Motor"]["value"]["motor.m1"]["data"],
                    log_vars["component"]["Motor"]["value"]["motor.m2"]["data"],
                    log_vars["component"]["Motor"]["value"]["motor.m3"]["data"],
                    log_vars["component"]["Motor"]["value"]["motor.m4"]["data"]
                )
            ])

            battery_voltage = np.array(log_vars["component"]["Battery"]["value"]["pm.vbatMV"]["data"])

            # index_0, index_1 = match_timeline(timeline_0, timeline_1)
            #
            # motor_PWM = data_time_alignment(motor_PWM[index_1:], timeline_1[index_1] - timeline_0[index_0])
            # battery_voltage = data_time_alignment(battery_voltage[index_1:], timeline_1[index_1] - timeline_0[index_0])

            gyro_data = np.array(gyro_data)


            motor_voltage = np.array([b * np.array(pwm) for b, pwm in zip(battery_voltage/1000, motor_PWM/65535)])

            I_xyz = [0.000023951, 0.000023951, 0.000032347]  # moment of inertia, kg/m^2
            l = (0.092 / 2) * np.sqrt(2)  # arm length
            h = -0.02  # -1 cm center of mass height
            K_V = 13  # rpm/mV
            C_T = 0.1  # depend on Propeller Shape
            rho = 1.225  # air pressure
            propeller_diameter = 0.051  # meter

            k_T = calculate_k_T(C_T, rho, propeller_diameter)

            actual_accel, predicted_accel, error = compute_angular_accel_error(timeline_0, gyro_data, motor_voltage,
                                                                               I_xyz, k_T, K_V, l, h)

            img_name = os.path.join(directory_name, "".join(f.split('.')[:-1]) + "_error")

            time_line = (np.array(timeline_0[2:]) - timeline_0[2])
            plot_general(img_name, time_line, error, line_names=["Roll", "Pitch", "Yaw"], x_label="Time(s)",
                         y_label="Angular Acc Residual (deg/s)")
            # Print the results
            # print("Actual Angular Acceleration:", actual_accel)
            # print("Predicted Angular Acceleration:", predicted_accel)
            # print("Error:", error)
