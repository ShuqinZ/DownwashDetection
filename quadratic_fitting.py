import os

import pandas as pd
import numpy as np
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from scipy.optimize import curve_fit
from util.data_process import *

project_root = Path(__file__).resolve().parent

dir_path = os.path.join(project_root, "metrics")

config = "x0_y0_z24_yaw0_TH35000_R0_P0_YR0"
directory_name = os.path.join(dir_path, config)

filepath = os.path.join(directory_name, "2025_02_25_11_34_0.json")
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

downwash_index = 0
for i, ts in enumerate(timeline_0):
    if ts < downwash_time:
        continue
    else:
        downwash_index = i
        break

# Define the multi-motor quadratic model
def multi_motor_quadratic(V, a2, a1, a0, b2, b1, b0, c2, c1, c0, d2, d1, d0):
    V1, V2, V3, V4 = V
    return (
            a2 * V1 ** 2 + a1 * V1 + a0 +
            b2 * V2 ** 2 + b1 * V2 + b0 +
            c2 * V3 ** 2 + c1 * V3 + c0 +
            d2 * V4 ** 2 + d1 * V4 + d0
    )


# Prepare input and target variables
V_input = motor_voltage[: downwash_index].T
alpha_roll = gyro_data[: downwash_index, 0]
alpha_pitch = gyro_data[: downwash_index, 1]
alpha_yaw = gyro_data[: downwash_index, 2]

# Fit the quadratic model for each axis
popt_roll, _ = curve_fit(multi_motor_quadratic, V_input, alpha_roll, maxfev=10000)
popt_pitch, _ = curve_fit(multi_motor_quadratic, V_input, alpha_pitch, maxfev=10000)
popt_yaw, _ = curve_fit(multi_motor_quadratic, V_input, alpha_yaw, maxfev=10000)

# # Display coefficients
# print(f"Roll coefficients: {popt_roll}")
# print(f"Pitch coefficients: {popt_pitch}")
# print(f"Yaw coefficients: {popt_yaw}")

# V_input_exm = motor_voltage[:].T
# Generate predictions
predicted_alpha_roll = multi_motor_quadratic(V_input, *popt_roll)
predicted_alpha_pitch = multi_motor_quadratic(V_input, *popt_pitch)
predicted_alpha_yaw = multi_motor_quadratic(V_input, *popt_yaw)

# Compute evaluation metrics for Roll
rmse_roll = np.sqrt(mean_squared_error(alpha_roll, predicted_alpha_roll))
mae_roll = mean_absolute_error(alpha_roll, predicted_alpha_roll)
r2_roll = r2_score(alpha_roll, predicted_alpha_roll)

# Compute evaluation metrics for Pitch
rmse_pitch = np.sqrt(mean_squared_error(alpha_pitch, predicted_alpha_pitch))
mae_pitch = mean_absolute_error(alpha_pitch, predicted_alpha_pitch)
r2_pitch = r2_score(alpha_pitch, predicted_alpha_pitch)

# Compute evaluation metrics for Yaw
rmse_yaw = np.sqrt(mean_squared_error(alpha_yaw, predicted_alpha_yaw))
mae_yaw = mean_absolute_error(alpha_yaw, predicted_alpha_yaw)
r2_yaw = r2_score(alpha_yaw, predicted_alpha_yaw)


# Low RMSE & MAE → Model is accurate.
# R² close to 1 → Model explains the variance well.
# If R² is low (<0.5) → The model may be missing important factors.

print(f"Performance Metrics for Roll:")
print(f"RMSE: {rmse_roll:.6f}, MAE: {mae_roll:.6f}, R²: {r2_roll:.6f}")

print(f"\nPerformance Metrics for Pitch:")
print(f"RMSE: {rmse_pitch:.6f}, MAE: {mae_pitch:.6f}, R²: {r2_pitch:.6f}")

print(f"\nPerformance Metrics for Yaw:")
print(f"RMSE: {rmse_yaw:.6f}, MAE: {mae_yaw:.6f}, R²: {r2_yaw:.6f}")

import seaborn as sns
import matplotlib.pyplot as plt

# Compute residuals
residuals_roll = alpha_roll - predicted_alpha_roll
residuals_pitch = alpha_pitch - predicted_alpha_pitch
residuals_yaw = alpha_yaw - predicted_alpha_yaw

# Plot residual distributions
plt.figure(figsize=(12, 4))

for i, (residuals, label) in enumerate(zip(
        [residuals_roll, residuals_pitch, residuals_yaw], ["Roll", "Pitch", "Yaw"])):
    plt.subplot(1, 3, i + 1)
    sns.histplot(residuals, bins=30, kde=True)
    plt.axvline(0, color='red', linestyle='dashed')
    plt.title(f"Residual Distribution for {label}")

plt.tight_layout()
plt.show()

# # Plot results
# plt.figure(figsize=(10, 5))
#
# # Roll
# f, axes = plt.subplots(1, 3, figsize=(12, 8))
#
# axes[0].scatter(alpha_roll, predicted_alpha_roll, alpha=0.5)
# axes[0].plot([0, 1], [0, 1], transform=axes[0].transAxes)
# axes[0].set_xlabel("Measured Alpha Roll")
# axes[0].set_ylabel("Predicted Alpha Roll")
# axes[0].set_title("Measured vs Predicted Roll")
# axes[0].axis('equal')
#
# # Pitch
# axes[1].scatter(alpha_pitch, predicted_alpha_pitch, alpha=0.5)
# axes[1].plot([0, 1], [0, 1], transform=axes[1].transAxes)
# axes[1].set_xlabel("Measured Alpha Pitch")
# axes[1].set_ylabel("Predicted Alpha Pitch")
# axes[1].set_title("Measured vs Predicted Pitch")
# axes[1].axis('equal')
#
# # Yaw
# axes[2].scatter(alpha_yaw, predicted_alpha_yaw, alpha=0.5)
# axes[2].plot([0, 1], [0, 1], transform=axes[2].transAxes)
# axes[2].set_xlabel("Measured Alpha Yaw")
# axes[2].set_ylabel("Predicted Alpha Yaw")
# axes[2].set_title("Measured vs Predicted Yaw")
# axes[2].axis('equal')
#
# plt.tight_layout()
# plt.show()