import deltarobot
import numpy as np
import matplotlib.pyplot as plt
import json

fig = plt.figure(figsize=(7, 7), dpi=100)
ax = fig.add_subplot(111, projection="3d")
contours = [[], [], []]

# delta = deltarobot.DeltaRobot()
# x = range(-300, 300, 5) # np.arange causes unexpected behaviour in the deltarobot class
# y = range(-300, 300, 5)
# z = range(-500, 0, 10)
# valid_points = [[], [], []]
#
# for z_val in z:
#     for y_val in y:
#         for x_val in x:
#             if delta.calculateIPK((x_val, y_val, z_val)):
#                 valid_points[0].append(x_val)
#                 valid_points[1].append(y_val)
#                 valid_points[2].append(z_val)
#
# for z_cor in z:
#     z_indices = [i for i, x in enumerate(valid_points[2]) if x == z_cor]  # finding a plane of given Z
#     for y_cor in y:
#         temp_x = {}
#         y_indices = []
#         temp_points = []
#         for index in z_indices:
#             if valid_points[1][index] == y_cor:
#                 y_indices.append(index)  # finding a line of given Z and Y
#                 temp_x[index] = valid_points[0][index]  # adding that point with its index to a temporary list
#         if temp_x:
#             min_index = min(temp_x, key=temp_x.get)
#             max_index = max(temp_x, key=temp_x.get)
#             # print(f"{min_index = }, {max_index = }")
#             contours[0].append(valid_points[0][min_index])
#             contours[0].append(valid_points[0][max_index])
#             contours[1].append(valid_points[1][min_index])
#             contours[1].append(valid_points[1][max_index])
#             contours[2].append(valid_points[2][min_index])
#             contours[2].append(valid_points[2][max_index])
#
# data = {
#     "x": [],
#     "y": [],
#     "z": [],
# }
#
# for point in contours[0]:
#     data["x"].append(point)
# for point in contours[1]:
#     data["y"].append(point)
# for point in contours[2]:
#     data["z"].append(point)
#
# contour_json = json.dumps(data)
# with open("working area/contours_light.txt", 'w') as file:
#     file.write(contour_json)

# with open("working area/contours_dense.txt", 'r') as file:
#     contours_json = json.load(file)
#     contours[0] = contours_json["x"]
#     contours[1] = contours_json["y"]
#     contours[2] = contours_json["z"]

with open("working area/contours_light.txt", 'r') as file:
    contours_json = json.load(file)
    contours[0] = contours_json["x"]
    contours[1] = contours_json["y"]
    contours[2] = contours_json["z"]

# ax.scatter(valid_points[0], valid_points[1], valid_points[2], s=0.05, alpha=1)
ax.scatter(contours[0], contours[1],contours[2], s=0.5, alpha=1)
# ax.plot(x, y, z)
# bx = fig.add_subplot(122)
# bx.scatter(contours[0], contours[2])
plt.show()
