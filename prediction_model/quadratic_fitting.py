import os

import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
from util.data_process import *

import seaborn as sns
import matplotlib as mpl
import matplotlib.pyplot as plt

from util.prediction import *

mpl.use('macosx')

project_root = Path(__file__).resolve().parent
dir_path = os.path.join(project_root, "..", "metrics")
config = "baseline"
directory_name = os.path.join(dir_path, config)
filepath = os.path.join(directory_name, "stable_flight.json")

V_input, alpha_roll, alpha_pitch, alpha_yaw = load_data_values(filepath, downwash=False, pre_downwash=True)

# Fit the quadratic model for each axis
popt_roll, _ = curve_fit(multi_motor_quadratic, V_input, alpha_roll, maxfev=10000)
popt_pitch, _ = curve_fit(multi_motor_quadratic, V_input, alpha_pitch, maxfev=10000)
popt_yaw, _ = curve_fit(multi_motor_quadratic, V_input, alpha_yaw, maxfev=10000)

# # Display coefficients
# print(f"Roll coefficients: {popt_roll}")
# print(f"Pitch coefficients: {popt_pitch}")
# print(f"Yaw coefficients: {popt_yaw}")

# # V_input_exm = motor_voltage[:].T
# # Generate predictions
predicted_alpha_roll = multi_motor_quadratic(V_input, *popt_roll)
predicted_alpha_pitch = multi_motor_quadratic(V_input, *popt_pitch)
predicted_alpha_yaw = multi_motor_quadratic(V_input, *popt_yaw)

# Compute evaluation metrics for Roll
# rmse_roll = np.sqrt(mean_squared_error(alpha_roll, predicted_alpha_roll))
# mae_roll = mean_absolute_error(alpha_roll, predicted_alpha_roll)
# r2_roll = r2_score(alpha_roll, predicted_alpha_roll)
#
# # Compute evaluation metrics for Pitch
# rmse_pitch = np.sqrt(mean_squared_error(alpha_pitch, predicted_alpha_pitch))
# mae_pitch = mean_absolute_error(alpha_pitch, predicted_alpha_pitch)
# r2_pitch = r2_score(alpha_pitch, predicted_alpha_pitch)
#
# # Compute evaluation metrics for Yaw
# rmse_yaw = np.sqrt(mean_squared_error(alpha_yaw, predicted_alpha_yaw))
# mae_yaw = mean_absolute_error(alpha_yaw, predicted_alpha_yaw)
# r2_yaw = r2_score(alpha_yaw, predicted_alpha_yaw)
#
#
# # Low RMSE & MAE → Model is accurate.
# # R² close to 1 → Model explains the variance well.
# # If R² is low (<0.5) → The model may be missing important factors.
#
# print(f"Performance Metrics for Roll:")
# print(f"RMSE: {rmse_roll:.6f}, MAE: {mae_roll:.6f}, R²: {r2_roll:.6f}")
#
# print(f"\nPerformance Metrics for Pitch:")
# print(f"RMSE: {rmse_pitch:.6f}, MAE: {mae_pitch:.6f}, R²: {r2_pitch:.6f}")
#
# print(f"\nPerformance Metrics for Yaw:")
# print(f"RMSE: {rmse_yaw:.6f}, MAE: {mae_yaw:.6f}, R²: {r2_yaw:.6f}")
#
#
# # Compute residuals
# residuals_roll = alpha_roll - predicted_alpha_roll
# residuals_pitch = alpha_pitch - predicted_alpha_pitch
# residuals_yaw = alpha_yaw - predicted_alpha_yaw
#
#
# # Plot residual distributions
# plt.figure(figsize=(12, 4))
#
# for i, (residuals, label) in enumerate(zip(
#         [residuals_roll, residuals_pitch, residuals_yaw], ["Roll", "Pitch", "Yaw"])):
#     plt.subplot(1, 3, i + 1)
#     sns.histplot(residuals, bins=30, kde=True)
#     plt.axvline(0, color='red', linestyle='dashed')
#     plt.title(f"Residual Distribution for {label}")
#
# plt.tight_layout()
# plt.show()
#
# Plot results
# plt.figure(figsize=(10, 5))

# Roll
# f, axes = plt.subplots(1, 3, figsize=(12, 8))
#
# axes[0].scatter(alpha_roll, predicted_alpha_roll, alpha=0.5)
# axes[0].plot([0, 1], [0, 1], transform=axes[0].transAxes)
# axes[0].set_xlabel("Measured Alpha Roll")
# axes[0].set_ylabel("Predicted Alpha Roll")
# axes[0].set_title("Measured vs Predicted Roll")
# axes[0].set_aspect('equal', adjustable='box')
#
# # Pitch
# axes[1].scatter(alpha_pitch, predicted_alpha_pitch, alpha=0.5)
# axes[1].plot([0, 1], [0, 1], transform=axes[1].transAxes)
# axes[1].set_xlabel("Measured Alpha Pitch")
# axes[1].set_ylabel("Predicted Alpha Pitch")
# axes[1].set_title("Measured vs Predicted Pitch")
# axes[1].set_aspect('equal', adjustable='box')
#
# # Yaw
# axes[2].scatter(alpha_yaw, predicted_alpha_yaw, alpha=0.5)
# axes[2].plot([0, 1], [0, 1], transform=axes[2].transAxes)
# axes[2].set_xlabel("Measured Alpha Yaw")
# axes[2].set_ylabel("Predicted Alpha Yaw")
# axes[2].set_title("Measured vs Predicted Yaw")
# axes[2].set_aspect('equal', adjustable='box')
#
# plt.tight_layout()
# plt.show()

project_root = Path(__file__).resolve().parent
dir_path = os.path.join(project_root, "..", "metrics")
config = "x8_y8_z24_yaw0_TH35000_R0_P0_YR0"
directory_name = os.path.join(dir_path, config)
filepath = os.path.join(directory_name, "2025_02_25_11_45_1.json")

pre_downwash_V, pre_downwash_roll, pre_downwash_pitch, pre_downwash_yaw = load_data_values(filepath, downwash=True, pre_downwash=True)

# Generate predictions
predicted_pre_roll = multi_motor_quadratic(pre_downwash_V, *popt_roll)
predicted_pre_pitch = multi_motor_quadratic(pre_downwash_V, *popt_pitch)
predicted_pre_yaw = multi_motor_quadratic(pre_downwash_V, *popt_yaw)


post_downwash_V, post_downwash_roll, post_downwash_pitch, post_downwash_yaw = load_data_values(filepath, downwash=True, pre_downwash=False)

# Generate predictions
predicted_post_roll = multi_motor_quadratic(post_downwash_V, *popt_roll)
predicted_post_pitch = multi_motor_quadratic(post_downwash_V, *popt_pitch)
predicted_post_yaw = multi_motor_quadratic(post_downwash_V, *popt_yaw)

# Roll
f, axes = plt.subplots(1, 3, figsize=(18, 8))

axes[0].scatter(pre_downwash_roll, predicted_pre_roll, alpha=0.5, color='b', label="No Downwash")
axes[0].scatter(post_downwash_roll, predicted_post_roll, alpha=0.5, color='r', label="With Downwash")
axes[0].plot([0, 1], [0, 1], transform=axes[0].transAxes)
axes[0].set_xlabel("Measured Alpha Roll")
axes[0].set_ylabel("Predicted Alpha Roll")
axes[0].set_title("Measured vs Predicted Roll")
axes[1].set_aspect(1./axes[1].get_data_ratio())
axes[0].legend()

# Pitch
axes[1].scatter(pre_downwash_pitch, predicted_pre_pitch, alpha=0.5, color='b', label="No Downwash")
axes[1].scatter(post_downwash_pitch, predicted_post_pitch, alpha=0.5, color='r', label="With Downwash")
axes[1].plot([0, 1], [0, 1], transform=axes[1].transAxes)
axes[1].set_xlabel("Measured Alpha Pitch")
axes[1].set_ylabel("Predicted Alpha Pitch")
axes[1].set_title("Measured vs Predicted Pitch")
axes[1].set_aspect(1./axes[1].get_data_ratio())
axes[1].legend()

# Yaw
axes[2].scatter(pre_downwash_yaw, predicted_pre_yaw, alpha=0.5, color='b', label="No Downwash")
axes[2].scatter(post_downwash_yaw, predicted_post_yaw, alpha=0.5, color='r', label="With Downwash")
axes[2].plot([0, 1], [0, 1], transform=axes[2].transAxes)
axes[2].set_xlabel("Measured Alpha Yaw")
axes[2].set_ylabel("Predicted Alpha Yaw")
axes[2].set_title("Measured vs Predicted Yaw")
axes[2].legend()
axes[2].set_aspect(1./axes[2].get_data_ratio())

# plt.tight_layout()
plt.show()