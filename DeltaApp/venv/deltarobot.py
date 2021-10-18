import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import VerticesCoordinates


# angle limits used in limiting the movement
OUT_OF_RANGE_ANGLE_LOW = -0.5235987756
OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268


class OutOfRangeError(Exception):
    """Raised when angle values are out of range specified in the program"""
    pass

class DeltaRobot():
    
    def __init__(self):

        self.vert_coords = VerticesCoordinates.VerticesCoordinates()
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
                if Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2 < 0:
                    raise OutOfRangeError
                temp = 2 * np.arctan((-Fi[i] + np.sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)) / (Gi[i] - Ei[i]))
                if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                    self.fi[i] = temp
                else:
                    temp = 2 * np.arctan((-Fi[i] - np.sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)) / (Gi[i] - Ei[i]))
                    if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                        self.fi[i] = temp
                    else:
                        raise OutOfRangeError
                print("Fi ", i, "to", self.fi[i]*180/3.14)

        except RuntimeWarning:
            return
        except OutOfRangeError:
            print("The coordinates are out of range!")
            raise TypeError

        else:
            ##### FOR ANIMATIONS #####

            # coordinates of elbow joints in the base frame {B} [x,y,z]
            A1 = [0, -self.wb - self.Length * np.cos(self.fi[0]), -self.Length * np.sin(self.fi[0])]
            A2 = [np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(self.fi[1])), 1 / 2 * (self.wb + self.Length * np.cos(self.fi[1])),
                  -self.Length * np.sin(self.fi[1])]
            A3 = [-np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(self.fi[2])), 1 / 2 * (self.wb + self.Length * np.cos(self.fi[2])),
                  -self.Length * np.sin(self.fi[2])]

            self.vert_coords.X0 = [self.B1[0], A1[0], self.P1[0] + x]
            self.vert_coords.X1 = [self.B2[0], A2[0], self.P2[0] + x]
            self.vert_coords.X2 = [self.B3[0], A3[0], self.P3[0] + x]
            self.vert_coords.X3 = [self.P1[0] + x, self.P2[0] + x, self.P3[0] + x, self.P1[0] + x]
            
            self.vert_coords.Y0 = [self.B1[1], A1[1], self.P1[1] + y]
            self.vert_coords.Y1 = [self.B2[1], A2[1], self.P2[1] + y]
            self.vert_coords.Y2 = [self.B3[1], A3[1], self.P3[1] + y]
            self.vert_coords.Y3 = [self.P1[1] + y, self.P2[1] + y, self.P3[1] + y, self.P1[1] + y]

            self.vert_coords.Z0 = [self.B1[2], A1[2], self.P1[2] + z]
            self.vert_coords.Z1 = [self.B2[2], A2[2], self.P2[2] + z]
            self.vert_coords.Z2 = [self.B3[2], A3[2], self.P3[2] + z]
            self.vert_coords.Z3 = [self.P1[2] + z, self.P2[2] + z, self.P3[2] + z, self.P1[2] + z]

            # list of lists of coordinates, X_coord_list_combined has all the X coordinates needed to plot the delta model: motor joint X, elbow X, effector joint X - for all links
            self.vert_coords.coordinates_x = [self.vert_coords.X0, self.vert_coords.X1, self.vert_coords.X2, self.vert_coords.X3, x+self.TCP[0]]
            self.vert_coords.coordinates_y = [self.vert_coords.Y0, self.vert_coords.Y1, self.vert_coords.Y2, self.vert_coords.Y3, y+self.TCP[1]]
            self.vert_coords.coordinates_z = [self.vert_coords.Z0, self.vert_coords.Z1, self.vert_coords.Z2, self.vert_coords.Z3, z-self.TCP[2]]
