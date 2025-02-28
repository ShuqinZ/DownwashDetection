import numpy as np
from util.data_process import *
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from tqdm import tqdm
from sklearn.gaussian_process import GaussianProcessRegressor
from scipy.spatial.transform import Rotation as R


def multi_motor_quadratic(V, a2, a1, a0, b2, b1, b0, c2, c1, c0, d2, d1, d0):
    V1, V2, V3, V4 = V
    return (
            a2 * V1 ** 2 + a1 * V1 + a0 +
            b2 * V2 ** 2 + b1 * V2 + b0 +
            c2 * V3 ** 2 + c1 * V3 + c0 +
            d2 * V4 ** 2 + d1 * V4 + d0
    )


def evaluate_model(y_true, y_pred, label):
    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
    mae = mean_absolute_error(y_true, y_pred)
    r2 = r2_score(y_true, y_pred)
    print(f"{label} Performance:")
    print(f"  RMSE: {rmse:.6f}, MAE: {mae:.6f}, R²: {r2:.6f}\n")


def train_gpr_with_progress(gpr, X_train, y_train, batch_size=200):
    n_samples = len(X_train)
    trained_models = []

    with tqdm(total=n_samples, desc="Training GPR", unit="sample") as pbar:
        for i in range(0, n_samples, batch_size):
            X_batch = X_train[i:i + batch_size]
            y_batch = y_train[i:i + batch_size]

            # Clone and train GPR on batch
            gpr_batch = GaussianProcessRegressor(kernel=gpr.kernel, n_restarts_optimizer=5, alpha=0.1)
            gpr_batch.fit(X_batch, y_batch)

            trained_models.append(gpr_batch)
            pbar.update(len(X_batch))

    return trained_models


def predict_gpr_ensemble(gpr_models, X_test):
    """
    Predict using multiple GPR models and aggregate the results.

    Parameters:
    - gpr_models (list): List of trained GPR models.
    - X_test (numpy array): Test set features.

    Returns:
    - numpy array: Averaged predictions across models.
    """
    preds = np.array([gpr.predict(X_test) for gpr in gpr_models])  # Shape: (num_models, num_samples)
    return np.mean(preds, axis=0)  # Average over all models



def predict_orientation(voltage, quat, omega, I, K_V, k_t, k_d, dt, arm_length):
    rpm = K_V * voltage * 0.75

    I_inv = np.linalg.inv(I)
    # === Compute Forces and Torques ===
    thrust = np.zeros((4, 3))  # Initialize force array (4 rotors, 3D forces)
    thrust[:, 2] = k_t * ((rpm * 2 * np.pi/60) ** 2)

    torque_drag = k_d * (rpm ** 2)  # Torque due to drag

    # Define rotor positions (based on new order: 1=Front Right, 2=Rear Right, 3=Rear Left, 4=Front Left)
    rotor_positions = np.array([
        [arm_length, arm_length, 0],   # Front Right (Motor 1)
        [arm_length, -arm_length, 0],  # Rear Right (Motor 2)
        [-arm_length, -arm_length, 0],  # Rear Left (Motor 3)
        [-arm_length, arm_length, 0],  # Front Left (Motor 4)
    ])

    # Rotation directions (+1 for CCW, -1 for CW) for standard quadcopter configuration
    spin_direction = np.array([-1, 1, -1, 1])  # Motors 1 & 3 CW, 2 & 4 CCW

    # Compute torque contributions
    torques = np.sum(cross(rotor_positions, thrust), axis=0)

    # cross(rotor_positions, np.vstack([np.zeros(3), np.zeros(3), np.zeros(3), thrust]).T).sum(axis=0)
    torques[2] += np.sum(spin_direction * torque_drag)  # Yaw torque

    # === Compute Angular Acceleration ===
    angular_acceleration = I_inv @ (torques - cross(omega, I @ omega))

    # === Update Angular Velocity ===
    omega_next = omega + angular_acceleration * dt

    # # === Update Orientation using Quaternion Integration ===
    # omega_quat = np.hstack(([0], omega_next))  # Convert to quaternion format
    #
    # # Quaternion derivative (q_dot = 0.5 * q * omega_quat)
    # q_dot = 0.5 * R.from_quat(quat).as_matrix() @ omega_quat[1:]
    #
    # # Integrate quaternion
    # quat_next = quat + q_dot * dt
    # quat_next /= np.linalg.norm(quat_next)  # Normalize quaternion

    return omega_next
    # # === Output Next Orientation ===
    # print("Next Orientation (Quaternion):", quat_next)
    # print("Next Angular Velocity:", omega_next)


def get_k_T(C_T, rho, propeller_diameter):
    """
    Blade Element Theory Approximation
    Calculate the thrust coefficient k_T based on the momentum theory.

    Parameters:
    - C_T (float): Thrust coefficient (depends on propeller shape).
    - rho (float): Air density (kg/m^3).
    - propeller_diameter (float): Diameter of the propeller in meters.

    Returns:
    - k_T (float): Thrust coefficient in N/(rad/s)^2.
    """
    A = (np.pi * propeller_diameter ** 2)/4  # Disk area

    k_T = C_T * rho * A * ((2*np.pi)/60) ** 2
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
        # Compute thrust using the quadratic model
        return k_T * rpm ** 2

    if isinstance(V, np.ndarray):
        thrust = []
        for v in V:
            thrust.append(calculate_thrust(k_T, K_V, v))
    else:
        # Compute thrust using the quadratic model
        thrust = calculate_thrust(k_T, K_V, V)

    return thrust
