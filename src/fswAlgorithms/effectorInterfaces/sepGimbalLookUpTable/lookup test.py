import numpy as np
from scipy.optimize import curve_fit

# Given data
data = np.array([
    [17.0,  0.0,  31.139, 31.139],
    [17.0, -0.5,  27.335, 34.445],
    [17.0, -1.0,  22.700, 37.396],
    [17.0, -1.5,  16.357, 40.066],
    [17.0, -2.0,   1.642, 25.821],
    [16.5,  2.5,  48.195, 10.725],
    [16.5,  2.0,  46.112, 20.075],
    [16.5,  1.5,  43.908, 25.447],
    [16.5,  1.0,  41.553, 29.630],
    [16.5,  0.5,  39.011, 33.149],
    [16.5,  0.0,  36.233, 36.233],
    [16.5, -0.5,  33.149, 39.011],
    [16.5, -1.0,  29.630, 41.553],
    [16.5, -1.5,  25.447, 43.908]
])

# Extracting tilt and tip angles
x1 = data[:, 0]  # Tilt
x2 = data[:, 1]  # Tip

# Extracting motor1 and motor2 angles
y1 = data[:, 2]  # Motor1
y2 = data[:, 3]  # Motor2

# Define the 2D polynomial function
def polynomial_surface(x1, x2, a0, a1, a2, a3, a4, a5):
    return a0 + a1 * x1 + a2 * x2 + a3 * x1**2 + a4 * x1 * x2 + a5 * x2**2

# Fitting the polynomial surface to motor1 data
popt1, pcov1 = curve_fit(polynomial_surface, (x1, x2), y1)

# Fitting the polynomial surface to motor2 data
popt2, pcov2 = curve_fit(polynomial_surface, (x1, x2), y2)

# Printing coefficients
print("Coefficients for Motor1 (a0, a1, a2, a3, a4, a5):", popt1)
print("Coefficients for Motor2 (a0, a1, a2, a3, a4, a5):", popt2)
