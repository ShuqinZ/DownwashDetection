import os
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
from sklearn.preprocessing import StandardScaler
from xgboost import XGBRegressor
from sklearn.model_selection import train_test_split
import seaborn as sns
import matplotlib.pyplot as plt
from util.data_process import *
from util.prediction import *

# Define the quadratic function for multi-motor input


# Load dataset
project_root = Path(__file__).resolve().parent
dir_path = os.path.join(project_root, "..", "metrics")
config = "baseline"
directory_name = os.path.join(dir_path, config)
filepath = os.path.join(directory_name, "stable_flight.json")

V_input, alpha_roll, alpha_pitch, alpha_yaw = load_data_values(filepath)

# Fit quadratic model for each axis
popt_roll, _ = curve_fit(multi_motor_quadratic, V_input, alpha_roll, maxfev=10000)
popt_pitch, _ = curve_fit(multi_motor_quadratic, V_input, alpha_pitch, maxfev=10000)
popt_yaw, _ = curve_fit(multi_motor_quadratic, V_input, alpha_yaw, maxfev=10000)

# Generate predictions using the quadratic model
predicted_alpha_roll = multi_motor_quadratic(V_input, *popt_roll)
predicted_alpha_pitch = multi_motor_quadratic(V_input, *popt_pitch)
predicted_alpha_yaw = multi_motor_quadratic(V_input, *popt_yaw)

# Compute residuals
residuals_roll = alpha_roll - predicted_alpha_roll
residuals_pitch = alpha_pitch - predicted_alpha_pitch
residuals_yaw = alpha_yaw - predicted_alpha_yaw

scaler = StandardScaler()
residuals_roll_scaled = scaler.fit_transform(residuals_roll.reshape(-1, 1)).flatten()
residuals_pitch_scaled = scaler.fit_transform(residuals_pitch.reshape(-1, 1)).flatten()
residuals_yaw_scaled = scaler.fit_transform(residuals_yaw.reshape(-1, 1)).flatten()

# Split into training and testing
X_train, X_test, y_train_roll, y_test_roll = train_test_split(V_input.T, residuals_roll, test_size=0.2, random_state=42)
X_train, X_test, y_train_pitch, y_test_pitch = train_test_split(V_input.T, residuals_pitch, test_size=0.2, random_state=42)
X_train, X_test, y_train_yaw, y_test_yaw = train_test_split(V_input.T, residuals_yaw, test_size=0.2, random_state=42)

# Train XGBoost on residuals
xgb_roll = XGBRegressor(n_estimators=100, learning_rate=0.05, max_depth=3)
xgb_pitch = XGBRegressor(n_estimators=100, learning_rate=0.05, max_depth=3)
xgb_yaw = XGBRegressor(n_estimators=100, learning_rate=0.05, max_depth=3)

xgb_roll.fit(X_train, y_train_roll)
xgb_pitch.fit(X_train, y_train_pitch)
xgb_yaw.fit(X_train, y_train_yaw)

# Predict residuals
y_pred_residuals_roll = scaler.inverse_transform(xgb_roll.predict(X_test).reshape(-1, 1)).flatten()
y_pred_residuals_pitch = scaler.inverse_transform(xgb_pitch.predict(X_test).reshape(-1, 1)).flatten()
y_pred_residuals_yaw = scaler.inverse_transform(xgb_yaw.predict(X_test).reshape(-1, 1)).flatten()

# Combine Quadratic + XGBoost Predictions
y_pred_hybrid_roll = multi_motor_quadratic(X_test.T, *popt_roll) + y_pred_residuals_roll
y_pred_hybrid_pitch = multi_motor_quadratic(X_test.T, *popt_pitch) + y_pred_residuals_pitch
y_pred_hybrid_yaw = multi_motor_quadratic(X_test.T, *popt_yaw) + y_pred_residuals_yaw

# Evaluate hybrid model

evaluate_model(y_test_roll, y_pred_hybrid_roll, "Hybrid Model - Roll")
evaluate_model(y_test_pitch, y_pred_hybrid_pitch, "Hybrid Model - Pitch")
evaluate_model(y_test_yaw, y_pred_hybrid_yaw, "Hybrid Model - Yaw")

# Plot residuals before & after XGBoost correction
plt.figure(figsize=(12, 4))
for i, (residuals, corrected, label) in enumerate(zip(
    [residuals_roll, residuals_pitch, residuals_yaw],
    [y_pred_hybrid_roll - y_test_roll, y_pred_hybrid_pitch - y_test_pitch, y_pred_hybrid_yaw - y_test_yaw],
    ["Roll", "Pitch", "Yaw"]
)):
    plt.subplot(1, 3, i+1)
    sns.histplot(residuals, bins=30, kde=True, label="Before XGBoost", color="blue", alpha=0.5)
    sns.histplot(corrected, bins=30, kde=True, label="After XGBoost", color="red", alpha=0.5)
    plt.axvline(0, color='black', linestyle='dashed')
    plt.title(f"Residuals for {label}")
    plt.legend()

plt.tight_layout()
plt.show()


# Plot results
plt.figure(figsize=(10, 5))

# Roll
f, axes = plt.subplots(1, 3, figsize=(12, 8))

axes[0].scatter(y_test_roll, y_pred_hybrid_roll, alpha=0.5)
axes[0].plot([0, 1], [0, 1], transform=axes[0].transAxes)
axes[0].set_xlabel("Measured Alpha Roll")
axes[0].set_ylabel("Predicted Alpha Roll")
axes[0].set_title("Measured vs Predicted Roll")
axes[0].set_aspect('equal', adjustable='box')

# Pitch
axes[1].scatter(y_test_pitch, y_pred_hybrid_pitch, alpha=0.5)
axes[1].plot([0, 1], [0, 1], transform=axes[1].transAxes)
axes[1].set_xlabel("Measured Alpha Pitch")
axes[1].set_ylabel("Predicted Alpha Pitch")
axes[1].set_title("Measured vs Predicted Pitch")
axes[1].set_aspect('equal', adjustable='box')

# Yaw
axes[2].scatter(y_test_yaw, y_pred_hybrid_yaw, alpha=0.5)
axes[2].plot([0, 1], [0, 1], transform=axes[2].transAxes)
axes[2].set_xlabel("Measured Alpha Yaw")
axes[2].set_ylabel("Predicted Alpha Yaw")
axes[2].set_title("Measured vs Predicted Yaw")
axes[2].set_aspect('equal', adjustable='box')

plt.tight_layout()
plt.show()