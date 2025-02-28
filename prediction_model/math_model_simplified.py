import numpy as np
from util import logger
from util.data_process import *
from matplotlib.colors import TABLEAU_COLORS, same_color
from util.prediction import *

if __name__ == '__main__':

    # configs = ["x0_y0_z24_yaw0_TH35000_R0_P0_YR0"]

    project_root = Path(__file__).resolve().parent

    dir_path = os.path.join(project_root, "../metrics")

    configs = list_folder(dir_path)

    for config in configs:
        directory_name = os.path.join(dir_path, config)
        file_names = list_files(directory_name)

        for f in file_names:
            if f.split('.')[-1] != "json":
                continue

            filepath = os.path.join(directory_name, f)

            timeline, motor_voltage, gyro_data, downwash_time = load_log(filepath)
            
            I_xyz = np.diag([2.3951e-5, 2.3951e-5, 3.2347e-5])  # moment of inertia, kg/m^2
            arm_length = (0.092 / 2) * np.sqrt(2)  # arm length
            h = -0.01  # -1 cm center of mass height
            m = 0.035
            K_V = 13000  # rpm/V
            C_T = 0.08  # depend on Propeller Shape
            rho = 1.225  # air pressure
            propeller_diameter = 0.051  # meter
            k_d = 0
            k_t = 1e-9

            error = []
            angular_velocity = [[] for _ in range(3)]
            for axis in range(3):
                # Compute actual angular acceleration (numerical derivative of gyro readings)
                angular_velocity[axis] = np.diff(gyro_data[:, axis]) / np.diff(timeline)  # Angular velocity

            angular_velocity = np.array(angular_velocity).T

            for i, data in enumerate(zip(np.diff(timeline), motor_voltage[1:], gyro_data[1:], angular_velocity)):
                dt, voltage, orientation, omega = data
                if i >= len(angular_velocity)-1:
                    break
                quat = euler_to_quaternion(orientation)
                predict_angular_v = predict_orientation(voltage, quat, omega, I_xyz, K_V, k_t, k_d, dt, arm_length)

                error.append(np.array(angular_velocity[i+1]) - np.array(predict_angular_v))

            img_name = os.path.join(directory_name, "".join(f.split('.')[:-1]) + "_error")


            downwash_time = downwash_time - timeline[1]

            timeline = (np.array(timeline[2:]) - timeline[2])

            error = np.array(error).T

            fig, axis = plt.subplots(nrows=2, figsize=(12, 8))
            axis[0].axvline(downwash_time, color='grey', linestyle="--", label="Downwash Start Time")

            for e, rpy, color in zip(error, ["Roll", "Pitch", "Yaw"], TABLEAU_COLORS):
                change_point = bcpd_window(e, 1)
                if change_point:
                    axis[0].axvline(timeline[change_point[0]-1], color='red', linestyle="--", label=f"{rpy} Detected Time",
                                c=color)

            plot_general("", timeline, error, line_names=["Roll", "Pitch", "Yaw"], x_label="Time(s)",
                         y_label="Angular Velocity Residual (rad/s)", x_lim=[timeline[0], timeline[-1]],
                         plot=[fig, axis[0]])

            total_error = [sum(abs(x) for x in values) for values in zip(*error)]
            change_point = bcpd_window(total_error, 1)

            axis[1].axvline(downwash_time, color='grey', linestyle="--", label="Downwash Start Time")
            axis[1].axvline(timeline[change_point[0]-1], color='red', linestyle="--",
                            label="Detected Downwash Start Time")

            plot_general("", timeline, total_error, line_names=["Sum"], x_label="Time(s)",
                         y_label="Total Angular Velocity Residual (rad/s)", x_lim=[timeline[0], timeline[-1]],
                         plot=[fig, axis[1]])
            plt.tight_layout()
            plt.savefig(f'{img_name}.png', dpi=300)
            logger.info(f'SAVED: {img_name}.png')
