# import numpy as np
# import os
# from scipy.optimize import curve_fit
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import pandas as pd
# from scipy.interpolate import griddata
# from scipy.interpolate import Rbf
# from scipy.interpolate import LinearNDInterpolator

# # Read CSV file, skipping the first column
# csvfile = pd.read_csv('platformAngle_motorAngle_small.csv', header=None)

# # Define the desired tip and tilt angles
# desired_tip = 3.1
# desired_tilt = -23.6

# # #Access data using iloc to avoid FutureWarning
# # x1 = csvfile.iloc[:, 1]  # Access data in the second column
# # x2 = csvfile.iloc[:, 2]  # Access data in the third column
# # y1 = csvfile.iloc[:, 3]  # Access data in the fourth column
# # y2 = csvfile.iloc[:, 4]  # Access data in the fifth column
# # Initialize lists to store the smallest distances and corresponding row indices

# # Access data of tip & tilt
# x = csvfile.iloc[:, 1]  # Access data in the second column (tip angles)
# y = csvfile.iloc[:, 2]  # Access data in the third column (tilt angles)

# smallest_distances = []
# smallest_index = []

# predictedMotor1 = None
# predictedMotor2 = None

# # Iterate through all rows & calculate distances
# for i in range(len(csvfile)):
#     norm = np.sqrt((desired_tip - x[i]) ** 2 + (desired_tilt - y[i]) ** 2)
    
#     if len(smallest_distances) < 4 or norm < max(smallest_distances):
#         # If there are less than 4 smallest distances recorded or the current distance is smaller than the largest recorded distance
#         # Add the current distance and its corresponding index to the lists
#         if len(smallest_distances) == 4:
#             # If there are already 4 smallest distances recorded, remove the largest one
#             max_distance_index = smallest_distances.index(max(smallest_distances))
#             del smallest_distances[max_distance_index]
#             del smallest_index[max_distance_index]
#         smallest_distances.append(norm)
#         smallest_index.append(i)

# # Store the smallest rows including motor 1 and motor 2 angles
# smallest_rows = csvfile.iloc[smallest_index]
# # Assign variable names for tip and tilt angles
# x1_tip = smallest_rows.iloc[0, 1]
# x2_tip = smallest_rows.iloc[2, 1]
# x3_tip = smallest_rows.iloc[0, 1]
# x4_tip = smallest_rows.iloc[2, 1]

# y1_tilt = smallest_rows.iloc[0, 2]
# y2_tilt = smallest_rows.iloc[1, 2]
# y3_tilt = smallest_rows.iloc[0, 2]
# y4_tilt = smallest_rows.iloc[1, 2]
# print('x1_tip',x1_tip,'x2_tip',x3_tip,'y3_tilt','x4_tip',x4_tip,'x2_tip',x2_tip,'y1_tilt',y1_tilt,'y2_tilt',y2_tilt,'y3_tilt',y3_tilt,'y4_tilt',y4_tilt)

# # Store the indices of the four smallest rows
# smallest_indices = smallest_rows.index.tolist()

# # Assign the smallest rows and their corresponding distances
# for i in range(len(smallest_rows)):
#     row_index = smallest_indices[i]
#     tip = smallest_rows.iloc[i, 1]
#     tilt = smallest_rows.iloc[i, 2]
#     motor1 = smallest_rows.iloc[i, 3]
#     motor2 = smallest_rows.iloc[i, 4]

#     # Assigning variables for each value dynamically based on the index
#     if i == 0:
#         motor1_Q11 = motor1
#         motor2_Q11 = motor2

#     elif i == 1:
#         motor1_Q21 = motor1
#         motor2_Q21 = motor2

#     elif i == 2:
#         motor1_Q12 = motor1
#         motor2_Q12 = motor2
#     elif i == 3:
#         motor1_Q22 = motor1
#         motor2_Q22 = motor2
#     print(f"Row {row_index}: Tip={tip}, Tilt={tilt}, Motor1={motor1}, Motor2={motor2}, Distance={smallest_distances[i]}")

# # if not (18.5 <= desired_tip <= -18.3) or not (27 <= desired_tilt <= -27):
# #     print('Desired tip or tilt angle is not within the gimbal angles bound')

# if (desired_tip, desired_tilt) in zip(x, y):
#     # If the desired tip and tilt angles exist, find the corresponding motor angles
#     index = (x == desired_tip) & (y == desired_tilt)
#     row = csvfile[index]
#     predictedMotor1 = row.iloc[0, 3]  # Motor 1 angle
#     predictedMotor2 = row.iloc[0, 4]  # Motor 2 angle
# else:
#     # Bilinear interpolation calculations for predicted motor 1
#     predictedMotor1 = (1 / ((x2_tip - x1_tip) * (y2_tilt - y1_tilt))) * (
#             motor1_Q11 * (x2_tip - desired_tip) * (y2_tilt - desired_tilt) +
#             motor1_Q21 * (desired_tip - x1_tip) * (y2_tilt - desired_tilt) +
#             motor1_Q12 * (x2_tip - desired_tip) * (desired_tilt - y1_tilt) +
#             motor1_Q22 * (desired_tip - x1_tip) * (desired_tilt - y1_tilt)
#     )

#     # Bilinear interpolation calculations for predicted motor 2
#     predictedMotor2 = (1 / ((x2_tip - x1_tip) * (y2_tilt - y1_tilt))) * (
#             motor2_Q11 * (x2_tip - desired_tip) * (y2_tilt - desired_tilt) +
#             motor2_Q21 * (desired_tip - x1_tip) * (y2_tilt - desired_tilt) +
#             motor2_Q12 * (x2_tip - desired_tip) * (desired_tilt - y1_tilt) +
#             motor2_Q22 * (desired_tip - x1_tip) * (desired_tilt - y1_tilt)
#     )

# # Extract the interpolation points and values
# points = np.column_stack((smallest_rows.iloc[:, 1], smallest_rows.iloc[:, 2]))
# values = np.column_stack((smallest_rows.iloc[:, 3], smallest_rows.iloc[:, 4]))

# # Create the interpolator
# interpolator = LinearNDInterpolator(points, values)

# # Evaluate the interpolator at the desired point
# predictedMotor11, predictedMotor22 = interpolator((desired_tip, desired_tilt))

# # Print the calculated predicted motor 1 and motor 2
# print("Predicted Motor 11 Angle:", predictedMotor11)
# print("Predicted Motor 22 Angle:", predictedMotor22)

# # # Calculate the barycentric coordinates of the desired point
# # u = (desired_tip - x1_tip) / (x2_tip - x1_tip)
# # v = (desired_tilt - y1_tilt) / (y2_tilt - y1_tilt)

# # # Interpolate the value at the desired point
# # predictedMotor1 = (1 - u) * (1 - v) * motor1_Q11 + u * (1 - v) * motor1_Q21 + (1 - u) * v * motor1_Q12 + u * v * motor1_Q22
# # predictedMotor2 = (1 - u) * (1 - v) * motor2_Q11 + u * (1 - v) * motor2_Q21 + (1 - u) * v * motor2_Q12 + u * v * motor2_Q22


# # # Print the calculated predicted motor 1 and motor 2
# # print("Predicted Motor 1 Angle:", predictedMotor1)
# # print("Predicted Motor 2 Angle:", predictedMotor2)


import numpy as np
import pandas as pd
from scipy.interpolate import LinearNDInterpolator
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

# Read CSV file, skipping the first column
csvfile = pd.read_csv('platformAngle_motorAngle_small.csv', header=None)

# Define the desired tip and tilt angles
desired_tip = -18.4
desired_tilt = -0.5

# Access/extract data of tip & tilt
x = csvfile.iloc[:, 1]  # Access data in the second column (tip angles)
y = csvfile.iloc[:, 2]  # Access data in the third column (tilt angles)

# Calculate distances to the desired point
distances = np.sqrt((x - desired_tip)**2 + (y - desired_tilt)**2)

# identify four closest points
closest_indices = np.argsort(distances)[:4]
smallest_rows = csvfile.iloc[closest_indices]

# Printing closest tip tilt angles
x_tips = smallest_rows.iloc[:, 1].values
y_tilts = smallest_rows.iloc[:, 2].values

print("Closest 4 Points for Interpolation:")
for i in range(4):
    print(f"Point {i+1}: Tip={x_tips[i]}, Tilt={y_tilts[i]}")

# Check for distinct points
if len(set(zip(x_tips, y_tilts))) < 4:
    raise ValueError("The chosen points do not form a valid grid for bilinear interpolation.")

# put angles in array and Extract the interpolation points and values
points = np.column_stack((x_tips, y_tilts))
z1_motor1 = smallest_rows.iloc[:, 3].values
z2_motor2 = smallest_rows.iloc[:, 4].values

print("Interpolation Points:")
print('points',points)
print("Values for Motor 1:", z1_motor1)
print("Values for Motor 2:", z2_motor2)

# Create the interpolator for both motor angles
interpolator_motor1 = LinearNDInterpolator(points, z1_motor1)
interpolator_motor2 = LinearNDInterpolator(points, z2_motor2)

# Check if the desired point is within the convex hull of the points
from scipy.spatial import Delaunay

hull = Delaunay(points)
if not hull.find_simplex((desired_tip, desired_tilt)) >= 0:
    print("The desired point is outside the convex hull of the input points.")

# Evaluate the interpolator at the desired point
predictedMotor1 = interpolator_motor1((desired_tip, desired_tilt))
predictedMotor2 = interpolator_motor2((desired_tip, desired_tilt))

# Check if the interpolation result is a valid number not NaN
if np.isnan(predictedMotor1) or np.isnan(predictedMotor2):
    print("Interpolation resulted in NaN ")

# Plot the table of point
plt.show(block=True)
plt.scatter(x, y, color='blue', label='Data Points')
plt.scatter(x_tips, y_tilts, color='red', label='Interpolation Points')
plt.scatter(desired_tip, desired_tilt, color='green', label='Desired Point')
plt.xlabel('Tip Angle')
plt.ylim()

# Print the calculated predicted motor angles
print("Predicted Motor 1 Angle:", predictedMotor1)
print("Predicted Motor 2 Angle:", predictedMotor2)

