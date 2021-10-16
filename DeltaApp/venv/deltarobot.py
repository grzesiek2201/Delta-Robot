import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


#angle limits used in limiting the movement
OUT_OF_RANGE_ANGLE_LOW = -0.5235987756
OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268


class OutOfRangeError(Exception):
    """Raised when angle values are out of range specified in the program"""
    pass

class DeltaRobot():
    
    def __init__(self):
        
        # geometry of the robot
        self.sb = 567  # length of the Side of the Base triangle in mm (sb)
        self.sp = 76  # length of the Side of the effector (Plate) triangle in mm (sp)
        self.Length = 524  # length of the biceps in mm (L)
        self.length = 1244  # length of the forearm in mm (l)
        self.wb = 1 / 3 * np.sqrt(3) / 2 * self.sb  # distance from the center of the Base to the side of the triangle
        self.ub = 2 * self.wb  # distance from the center of the Base to the side of the triangle
        self.wp = 1 / 3 * np.sqrt(3) / 2 * self.sp  # distance from the center of the effector (Plate) to the vertex of the triangle
        self.up = 2 * self.wp
        self.a = self.wb - self.up
        self.b= self.sp / 2 - np.sqrt(3) / 2 * self.wb
        self.c = self.wp - 1 / 2 * self.wb
        self.TCP = [0, 0, 100] # offset for the TCP

        # variables
        self.fi = [0, 0, 0]

        # vectors for base joints Bi in base frame {B}
        self.B1 = [0, -self.wb, 0]
        self.B2 = [np.sqrt(3) / 2 * self.wb, 1 / 2 * self.wb, 0]
        self.B3 = [-np.sqrt(3) / 2 * self.wb, 1 / 2 * self.wb, 0]

        # vectors for effector joints Pi in effector frame {P}
        self.P1 = [0, -self.up, 0]
        self.P2 = [self.sp / 2, self.wp, 0]
        self.P3 = [-self.sp / 2, self.wp, 0]
        
        # coordinates of the base vertices (for animation)
        self.Bvx = [-self.sb / 2, 0, self.sb / 2, -self.sb / 2]
        self.Bvy = [-self.wb, self.ub, -self.wb, -self.wb]
        self.Bvz = [0, 0, 0, 0]
        

    def calculateIK(self, x, y, z):
        """Calculate inverse kinematics based on input coordinates. Returns a Tuple (x list, y list, z list) of lists of coordinates of points to plot"""

        # constants used to calculate Inverse Kinematics
        Ei = [2 * self.Length * (y + self.a),
              -self.Length * (np.sqrt(3) * (x + self.b) + y + self.c),
              self.Length * (np.sqrt(3) * (x - self.b) - y - self.c)]
        Fi = [2 * z * self.Length, 2 * z * self.Length, 2 * z * self.Length]
        Gi = [x ** 2 + y ** 2 + z ** 2 + self.a ** 2 + self.Length ** 2 + 2 * y * self.a - self.length ** 2,
              x ** 2 + y ** 2 + z ** 2 + self.b ** 2 + self.c ** 2 + self.Length ** 2 + 2 * x * self.b + 2 * y * self.c - self.length ** 2,
              x ** 2 + y ** 2 + z ** 2 + self.b ** 2 + self.c ** 2 + self.Length ** 2 - 2 * x * self.b + 2 * y * self.c - self.length ** 2]

        # try to calculate the joint angles, if they are out of limits, throw an OutOfRangeError
        try:
            for i in range(0, 3):
                temp = 2 * np.arctan((-Fi[i] + np.sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)) / (Gi[i] - Ei[i]))
                if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                    self.fi[i] = temp
                else:
                    temp = 2 * np.arctan((-Fi[i] - np.sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)) / (Gi[i] - Ei[i]))
                    if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                        self.fi[i] = temp
                    else:
                        print("Exception raised")
                        raise OutOfRangeError
                print("Fi ", i, "to", self.fi[i]*180/3.14)

        except OutOfRangeError:
            print("The coordinates are out of range!")

        else:
            ##### FOR ANIMATIONS #####

            # coordinates of elbow joints in the base frame {B} [x,y,z]
            A1 = [0, -self.wb - self.Length * np.cos(self.fi[0]), -self.Length * np.sin(self.fi[0])]
            A2 = [np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(self.fi[1])), 1 / 2 * (self.wb + self.Length * np.cos(self.fi[1])),
                  -self.Length * np.sin(self.fi[1])]
            A3 = [-np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(self.fi[2])), 1 / 2 * (self.wb + self.Length * np.cos(self.fi[2])),
                  -self.Length * np.sin(self.fi[2])]

            # # vectors from the base frame {B} to the effector frame {P}
            # link1 = [self.B1, A1, self.P1]
            # link2 = [self.B2, A2, self.P2]
            # link3 = [self.B3, A3, self.P3]

            X0 = [self.B1[0], A1[0], self.P1[0] + x]
            X1 = [self.B2[0], A2[0], self.P2[0] + x]
            X2 = [self.B3[0], A3[0], self.P3[0] + x]
            X3 = [self.P1[0] + x, self.P2[0] + x, self.P3[0] + x, self.P1[0] + x]
            
            Y0 = [self.B1[1], A1[1], self.P1[1] + y]
            Y1 = [self.B2[1], A2[1], self.P2[1] + y]
            Y2 = [self.B3[1], A3[1], self.P3[1] + y]
            Y3 = [self.P1[1] + y, self.P2[1] + y, self.P3[1] + y, self.P1[1] + y]

            Z0 = [self.B1[2], A1[2], self.P1[2] + z]
            Z1 = [self.B2[2], A2[2], self.P2[2] + z]
            Z2 = [self.B3[2], A3[2], self.P3[2] + z]
            Z3 = [self.P1[2] + z, self.P2[2] + z, self.P3[2] + z, self.P1[2] + z]

            # # Xi is a list of 'x' coordinates for link i, Yi is a list of 'y' coordinates for link i, Zi is a list of 'z' coordinates for link i
            # X1 = [link1[0][0], link1[1][0], link1[2][0] + x]  # added coordinates as to compensate for reference frame {P}
            # Y1 = [link1[0][1], link1[1][1], link1[2][1] + y]
            # Z1 = [link1[0][2], link1[1][2], link1[2][2] + z]
            # X2 = [link2[0][0], link2[1][0], link2[2][0] + x]
            # Y2 = [link2[0][1], link2[1][1], link2[2][1] + y]
            # Z2 = [link2[0][2], link2[1][2], link2[2][2] + z]
            # X3 = [link3[0][0], link3[1][0], link3[2][0] + x]
            # Y3 = [link3[0][1], link3[1][1], link3[2][1] + y]
            # Z3 = [link3[0][2], link3[1][2], link3[2][2] + z]
            # print(f"X1 coordinates: {X1}\nY1 coordinates: {Y1}\nZ1 coordinates: {Z1}\n")
            # 
            # # effector coordinates (only for visuals to close the triangles)
            # X4 = [link1[2][0] + x, link2[2][0] + x, link3[2][0] + x, link1[2][0] + x]
            # Y4 = [link1[2][1] + y, link2[2][1] + y, link3[2][1] + y, link1[2][1] + y]
            # Z4 = [link1[2][2] + z, link2[2][2] + z, link3[2][2] + z, link1[2][2] + z]

            # list of lists of coordinates, X_coord_list_combined has all the X coordinates needed to plot the delta model: motor joint X, elbow X, effector joint X - for all links
            coordinates_x = [X0, X1, X2, X3, x+self.TCP[0]]
            coordinates_y = [Y0, Y1, Y2, Y3, y+self.TCP[1]]
            coordinates_z = [Z0, Z1, Z2, Z3, z-self.TCP[2]]

            # # return a tuple of lists of lists
            return coordinates_x, coordinates_y, coordinates_z


    def plotDelta(self, X_coord_list_combined, Y_coord_list_combined, Z_coord_list_combined): ### PROBABLY DELETE THIS FROM THE CLASS ###
        """Plots the delta configuration based on a tuple with 3 lists of coordinates X Y Z"""
        plt.plot(X_coord_list_combined[0], Y_coord_list_combined[0], Z_coord_list_combined[0], 'black')             # Link 1
        plt.plot(X_coord_list_combined[1], Y_coord_list_combined[1], Z_coord_list_combined[1], 'black')             # Link 2
        plt.plot(X_coord_list_combined[2], Y_coord_list_combined[2], Z_coord_list_combined[2], 'black')             # Link 3
        plt.plot(X_coord_list_combined[3], Y_coord_list_combined[3], Z_coord_list_combined[3])                      # Effector
        plt.plot(X_coord_list_combined[4], Y_coord_list_combined[4], Z_coord_list_combined[4], 'ro')                # TCP
        plt.plot(self.Bvx, self.Bvy, self.Bvz)                                                                                     # Base

        plt.tight_layout()
        plt.show()


    def updatePlot(self, axis, x, y, z): ### PROBABLY DELETE THIS FROM THE CLASS ###
        plt.clf()
        axis = fig.add_subplot(111, projection='3d')
        plt.plot(X_coords[0], Y_coords[0], Z_coords[0])
        plt.plot(X_coords[1], Y_coords[1], Z_coords[1])
        plt.plot(X_coords[2], Y_coords[2], Z_coords[2])
        plt.plot(X_coords[3], Y_coords[3], Z_coords[3])
        plt.plot(X_coords[4], Y_coords[4], Z_coords[4])
        plt.draw()
        plt.pause(1)

#
# if __name__ == '__main__':
#
#     delta = DeltaRobot()
#
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection="3d")
#
#     input_coord_list = [float(input("X:")), float(input("Y:")), float(input("Z:"))]
#     X_coords, Y_coords, Z_coords = delta.calculateIK(input_coord_list[0], input_coord_list[1], input_coord_list[2])
#
#     delta.updatePlot(ax, X_coords, Y_coords, Z_coords)
#     input_coord_list = [float(input("X:")), float(input("Y:")), float(input("Z:"))]
#     X_coords, Y_coords, Z_coords = calculateIK(input_coord_list[0], input_coord_list[1], input_coord_list[2])
#     delta.updatePlot(ax, X_coords, Y_coords, Z_coords)
#     # end_of_program = False
#     # while(not end_of_program):
#     #     input_coord_list = [float(input("X:")), float(input("Y:")), float(input("Z:"))]
#     #     print(input_coord_list)
#     #     try:
#     #         X_coords, Y_coords, Z_coords = calculateIK(input_coord_list[0], input_coord_list[1], input_coord_list[2])
#     #         # print(XX, YY, ZZ)
#     #         # print(np.sqrt((XX[0][0] - XX[1][0]) ** 2 + (YY[0][0] - YY[1][0]) ** 2 + (
#     #         # ZZ[0][0] - ZZ[1][0]) ** 2))  # check the length of the bicesp and forearm (here wrong indexes)
#     #         ax = plt.axes(projection="3d")
#     #         plotDelta(X_coords, Y_coords, Z_coords)
#     #     except TypeError:
#     #         print("TypeError, didn't plot anything.")
#     #         pass
#     #  print('sb = ', sb, 'sp = ', sp, 'L = ', Length, 'l = ', length, 'wb = ', wb, 'ub = ', ub, 'wp = ', wp, 'up = ', up)
