import numpy as np
import bocpd
import matplotlib.pyplot as plt
from bocpd import GaussianUnknownMean

# Generate a simulated time series with a change at t=50
np.random.seed(42)
t_change = 50
n_samples = 100
data = np.concatenate([
    np.random.normal(0, 1, t_change),   # Before change
    np.random.normal(5, 1, n_samples - t_change)  # After change
])

# Define the model (Gaussian with unknown mean)
hazard = 1 / 50  # Assumed change point frequency
model = GaussianUnknownMean(variance=1)

# Run Bayesian Online Change Point Detection
R, maxes = bocpd.online_changepoint_detection(data, model, hazard)

# Plot the results
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(data, label="Residual Data")
plt.axvline(t_change, color='red', linestyle="--", label="True Change Point")
plt.legend()
plt.title("Simulated Residual Data")

plt.subplot(2, 1, 2)
plt.plot(R.sum(axis=0), label="Change Point Probability")
plt.axvline(t_change, color='red', linestyle="--", label="True Change Point")
plt.legend()
plt.title("Bayesian Online Change Point Detection (OBCPD)")
plt.xlabel("Time")

plt.tight_layout()
plt.show()
