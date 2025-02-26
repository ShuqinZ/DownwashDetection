import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from pathlib import Path
from util.data_process import load_data  # Ensure this function is correctly defined

### 1. Define LSTM Model ###
class LSTMModel(nn.Module):
    def __init__(self, input_size, hidden_size=128, num_layers=3, dropout=0.3):
        super(LSTMModel, self).__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True, dropout=dropout)
        self.fc = nn.Linear(hidden_size, 1)

    def forward(self, x):
        lstm_out, _ = self.lstm(x)
        return self.fc(lstm_out[:, -1, :])  # Use last timestep output



### 2. Define Feature Processing Functions ###
def add_lag_features(data, n_lags=3):
    """Adds lag features for time-series data."""
    df = pd.DataFrame(data)
    for lag in range(1, n_lags + 1):
        df = pd.concat([df, df.shift(lag)], axis=1)
    return df.fillna(0).values  # Fill NaNs from shifting

def create_time_series_data(data, target, n_lags=3):
    """
    Converts feature matrix and target into a time-series format for LSTM.

    Parameters:
    - data (numpy.ndarray): Feature matrix of shape (num_samples, num_features).
    - target (numpy.ndarray): Target values of shape (num_samples,).
    - n_lags (int): Number of past time steps to include.

    Returns:
    - X_seq (numpy.ndarray): Shape (num_samples - n_lags, n_lags, num_features).
    - y_seq (numpy.ndarray): Shape (num_samples - n_lags, 1).
    """
    X_seq, y_seq = [], []

    for i in range(n_lags, len(data)):  # Start at index `n_lags` to allow history
        X_seq.append(data[i - n_lags:i])  # Collect `n_lags` past steps
        y_seq.append(target[i])  # Predict current step

    return np.array(X_seq), np.array(y_seq)


def evaluate_model(y_true, y_pred, label):
    """Evaluates model performance using RMSE, MAE, and R²."""
    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
    mae = mean_absolute_error(y_true, y_pred)
    r2 = r2_score(y_true, y_pred)
    print(f"{label} Performance:")
    print(f"  RMSE: {rmse:.6f}, MAE: {mae:.6f}, R²: {r2:.6f}\n")


def moving_average_filter(data, window_size=5):
    """
    Applies a moving average filter to smooth noisy signals.

    Parameters:
    - data: numpy array (1D or 2D)
    - window_size: size of the moving window

    Returns:
    - Smoothed data (same shape as input)
    """
    return pd.DataFrame(data).rolling(window=window_size, center=True).mean().fillna(method='bfill').values


### 3. Load Dataset ###
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

### 4. Extract Downwash-Free Data ###
downwash_index = np.argmax(timeline_0 >= downwash_time)  # Find first index after downwash_time
timeline = timeline_0[:downwash_index]
V_input = motor_voltage[:downwash_index]
y_roll = gyro_data[:downwash_index, 0]
y_pitch = gyro_data[:downwash_index, 1]
y_yaw = gyro_data[:downwash_index, 2]
gyro_readings = gyro_data[:downwash_index]
### 5. Create Feature Set with Lagged Features ###
X_features = np.hstack([timeline.reshape(-1, 1), V_input, gyro_readings])  # Add battery voltage

# X_features = np.hstack([V_input, battery_voltage[:downwash_index].reshape(-1, 1)])  # Add battery voltage

# X_features_lagged = add_lag_features(X_features, n_lags=3).T   # Add time lags
# y_roll_lagged = add_lag_features(y_roll.reshape(-1, 1), n_lags=3).flatten()
# y_pitch_lagged = add_lag_features(y_pitch.reshape(-1, 1), n_lags=3).flatten()
# y_yaw_lagged = add_lag_features(y_yaw.reshape(-1, 1), n_lags=3).flatten()

### 6. Split Data ###
X_train, X_test, y_train_roll, y_test_roll = train_test_split(X_features, y_roll, test_size=0.2, random_state=42)
X_train, X_test, y_train_pitch, y_test_pitch = train_test_split(X_features, y_pitch, test_size=0.2, random_state=42)
X_train, X_test, y_train_yaw, y_test_yaw = train_test_split(X_features, y_yaw, test_size=0.2, random_state=42)


### 7. Convert to PyTorch Tensors ###
n_lags = 5  # Define number of lag steps

X_train_seq, y_train_seq = create_time_series_data(X_train, y_train_roll, n_lags)
X_test_seq, y_test_seq = create_time_series_data(X_test, y_test_roll, n_lags)

X_train_torch = torch.tensor(X_train_seq, dtype=torch.float32)
X_test_torch = torch.tensor(X_test_seq, dtype=torch.float32)
y_train_torch = torch.tensor(y_train_seq, dtype=torch.float32)
y_test_torch = torch.tensor(y_test_seq, dtype=torch.float32)

### 8. Train LSTM Model ###
lstm_model = LSTMModel(input_size=X_train_torch.shape[2])
optimizer = optim.Adam(lstm_model.parameters(), lr=0.001)
criterion = nn.MSELoss()

# Move to device (CPU/GPU)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
lstm_model.to(device)
X_train_torch, y_train_torch = X_train_torch.to(device), y_train_torch.to(device)
X_test_torch, y_test_torch = X_test_torch.to(device), y_test_torch.to(device)

# Training loop
for epoch in range(1000):
    optimizer.zero_grad()
    y_pred = lstm_model(X_train_torch)
    loss = criterion(y_pred, y_train_torch)
    loss.backward()
    optimizer.step()

    if epoch % 10 == 0:
        with torch.no_grad():
            test_loss = criterion(lstm_model(X_test_torch), y_test_torch).item()
        print(f"Epoch {epoch}: Test Loss = {test_loss:.6f}")

# Evaluate LSTM
y_pred_lstm = lstm_model(X_test_torch).detach().cpu().numpy().flatten()
evaluate_model(y_test_seq, y_pred_lstm, "LSTM - Roll")
