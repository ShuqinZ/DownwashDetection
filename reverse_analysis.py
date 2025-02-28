from util import logger
from util.data_process import *

from matplotlib.colors import TABLEAU_COLORS, same_color


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
    - k_V (float): V
    - V: Real time voltage

    Returns:
    - thrust (array): Estimated thrust values.
    """

    def calculate_thrust(k_T, K_V, v):
        # 1 RPM = 0.10472 rad/s
        rpm = K_V * v
        omega = rpm * 0.1047198
        # Compute thrust using the quadratic model
        return k_T * omega ** 2

    if isinstance(V, np.ndarray):
        thrust = []
        for v in V:
            thrust.append(calculate_thrust(k_T, K_V, v))
    else:
        # Compute thrust using the quadratic model
        thrust = calculate_thrust(k_T, K_V, V)

    return thrust


'''
                    ↑ (Front)
                T4 (CW)   T1 (CCW)
                ⬤-----------⬤
      Roll - <--   |       |    --> Roll+
                   |       |
                ⬤-----------⬤
                T3 (CCW)   T2 (CW)
                    ↓ (Back)

                Pitch -: Head Down
                Pitch +: Head up
                Yaw-: clockwise
                Yaw+: counter-cloclwise
'''
import numpy as np


def compute_angular_accel_error(time_line, gyro_data, motor_voltage, I_xyz, k_T, K_V, l, h, m, g=9.81):
    """
    Compute the error between actual and predicted angular acceleration for roll, pitch, and yaw.

    Parameters:
    - time_line (list): Time stamp in sec
    - gyro_data (numpy array): Gyroscope readings { 'roll': np.array, 'pitch': np.array, 'yaw': np.array } in rad.
    - motor_voltage (list or np.array): A list of tuples [(V1, V2, V3, V4), ...] with motor voltages per time step.
    - I_xyz (tuple): Moments of inertia (I_x, I_y, I_z) for roll, pitch, and yaw.
    - k_T (float): Thrust coefficient.
    - K_V (float): Motor speed constant.
    - l (float): Arm length from center to motor.
    - h (float): Center of mass height above thrust plane.
    - m (float): Mass of the drone.
    - g (float): Acceleration due to gravity (9.81 m/s²).

    Returns:
    - actual_accel (dict): Numerically computed angular acceleration for roll, pitch, and yaw.
    - predicted_accel (dict): Computed from the mathematical model.
    - error (dict): Difference between actual and predicted acceleration.
    """

    actual_gyro_rate = [[] for _ in range(3)]
    actual_accel = [[] for _ in range(3)]
    predicted_accel = [[] for _ in range(3)]

    I_x, I_y, I_z = I_xyz

    gyro_data = np.deg2rad(gyro_data)

    for axis in range(3):
        # Compute actual angular acceleration (numerical derivative of gyro readings)
        actual_gyro_rate[axis] = np.diff(gyro_data[:, axis]) / np.diff(time_line)  # Angular velocity
        actual_accel[axis] = np.diff(actual_gyro_rate[axis]) / np.diff(time_line[1:])  # Angular acceleration

    # time_line = time_line[1:]
    actual_gyro_rate = np.array(actual_gyro_rate)
    actual_accel = np.array(actual_accel)

    for i in range(len(actual_accel[0])):

        # Compute thrust for all four motors
        T1, T2, T3, T4 = get_thrust(k_T, K_V, motor_voltage[i])

        # Compute small-angle approximations
        theta = gyro_data[i, 1]  # Pitch angle (small approximation)
        phi = gyro_data[i, 0]  # Roll angle

        # Compute torques due to motor thrust
        tau_roll = l * (T1 - T2) * np.cos(phi) + h * (T1 + T2 + T3 + T4) * np.sin(phi)
        tau_pitch = l * (T4 - T3) * np.cos(theta) + h * (T1 + T2 + T3 + T4) * np.sin(theta)
        tau_yaw = l * ((T4 + T3) - (T1 + T2))

        predicted_ddot_roll = (tau_roll) / I_x
        predicted_ddot_pitch = (tau_pitch) / I_y
        predicted_ddot_yaw = (tau_yaw) / I_z  # No gravity effect on yaw

        # Store computed values
        predicted_accel[0].append(predicted_ddot_roll)
        predicted_accel[1].append(predicted_ddot_pitch)
        predicted_accel[2].append(predicted_ddot_yaw)

    # Convert lists to numpy arrays
    for axis in range(3):
        predicted_accel[axis] = np.array(predicted_accel[axis])

    # Compute errors
    residue = [np.array(predicted_accel[axis] - actual_accel[axis]) for axis in range(3)]

    return actual_accel, predicted_accel, residue


if __name__ == '__main__':

    # configs = ["x0_y0_z24_yaw0_TH35000_R0_P0_YR0"]

    project_root = Path(__file__).resolve().parent

    dir_path = os.path.join(project_root, "metrics")

    configs = list_folder(dir_path)

    for config in configs:
        directory_name = os.path.join(dir_path, config)
        file_names = list_files(directory_name)

        for f in file_names:
            if f.split('.')[-1] != "json":
                continue

            filepath = os.path.join(directory_name, f)
            log_vars = load_data(filepath)

            downwash_time = log_vars["Downwash_Start_Time"]

            timeline_0 = np.array(log_vars["timestamp"])
            gyro_data = np.array([
                [roll, pitch, yaw]
                for roll, pitch, yaw in zip(
                    log_vars["component"]["Gyro"]["value"]["stateEstimate.roll"]["data"],
                    log_vars["component"]["Gyro"]["value"]["stateEstimate.pitch"]["data"],
                    log_vars["component"]["Gyro"]["value"]["stateEstimate.yaw"]["data"]
                )
            ])

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

            motor_voltage = np.array([b * np.array(pwm) for b, pwm in zip(battery_voltage / 1000, motor_PWM / 65535)])

            I_xyz = [0.000023951, 0.000023951, 0.000032347]  # moment of inertia, kg/m^2
            l = (0.092 / 2) * np.sqrt(2)  # arm length
            h = -0.01  # -1 cm center of mass height
            m = 0.035
            K_V = 13000  # rpm/V
            C_T = 0.1  # depend on Propeller Shape
            rho = 1.225  # air pressure
            propeller_diameter = 0.051  # meter

            k_T = calculate_k_T(C_T, rho, propeller_diameter)

            actual_accel, predicted_accel, error = compute_angular_accel_error(timeline_0, gyro_data, motor_voltage,
                                                                               I_xyz, k_T, K_V, l, h, m)

            img_name = os.path.join(directory_name, "".join(f.split('.')[:-1]) + "_error")

            time_line = (np.array(timeline_0[2:]) - timeline_0[2])

            downwash_time = downwash_time - timeline_0[2]

            fig, axis = plt.subplots(nrows=2, figsize=(12, 8))
            axis[0].axvline(downwash_time, color='grey', linestyle="--", label="Downwash Start Time")

            for e, rpy, color in zip(error, ["Roll", "Pitch", "Yaw"], TABLEAU_COLORS):
                change_point = bcpd_window(e, 1)
                axis[0].axvline(time_line[change_point[0]], color='red', linestyle="--", label=f"{rpy} Detected Time",
                                c=color)

            plot_general("", time_line, error, line_names=["Roll", "Pitch", "Yaw"], x_label="Time(s)",
                         y_label="Angular Acc Residual (rad/s^2)", x_lim=[time_line[0], time_line[-1]],
                         plot=[fig, axis[0]])

            total_error = [sum(abs(x) for x in values) for values in zip(*error)]
            change_point = bcpd_window(total_error, 1)

            axis[1].axvline(downwash_time, color='grey', linestyle="--", label="Downwash Start Time")
            axis[1].axvline(time_line[change_point[0]], color='red', linestyle="--",
                            label="Detected Downwash Start Time")

            plot_general("", time_line, total_error, line_names=["Sum"], x_label="Time(s)",
                         y_label="Total Angular Acc Residual (rad/s^2)", x_lim=[time_line[0], time_line[-1]],
                         plot=[fig, axis[1]])
            plt.tight_layout()
            plt.savefig(f'{img_name}.png', dpi=300)
            logger.info(f'SAVED: {img_name}.png')
