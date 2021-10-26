import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import VerticesCoordinates


# angle limits used in limiting the movement
OUT_OF_RANGE_ANGLE_LOW = -0.5235987756
OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268


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

    def calculateIPK(self, x, y, z):
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
                    raise TypeError
                temp = 2 * np.arctan((-Fi[i] + np.sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)) / (Gi[i] - Ei[i]))
                if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                    self.fi[i] = temp
                else:
                    temp = 2 * np.arctan((-Fi[i] - np.sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)) / (Gi[i] - Ei[i]))
                    if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                        self.fi[i] = temp
                    else:
                        print("The coordinates are out of range!")
                        raise TypeError
                print("Fi ", i, "to", self.fi[i]*180/3.14)

        except RuntimeWarning:
            return

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

    def calculateFPK(self, fi1, fi2, fi3, radians=False):

        # Convert degrees to radians
        if not radians:
            fi1 = fi1 * 3.14 / 180
            fi2 = fi2 * 3.14 / 180
            fi3 = fi3 * 3.14 / 180

        # Calculate xi, yi, zi based on fi1, fi2, fi3
        x1 = 0
        y1 = -self.wb - self.Length * np.cos(fi1) + self.up
        z1 = -self.Length * np.sin(fi1)

        x2 = np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(fi2)) - self.sp / 2
        y2 = 1 / 2 * (self.wb + self.Length * np.cos(fi2)) - self.wp
        z2 = -self.Length * np.sin(fi2)

        x3 = -np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(fi3)) + self.sp / 2
        y3 = 1 / 2 * (self.wb + self.Length * np.cos(fi3)) - self.wp
        z3 = -self.Length * np.sin(fi3)

        r1 = self.length  # all three radia are the same length because the arms are symmetric
        r2 = self.length
        r3 = self.length

        # First substitutions
        a11 = 2 * (x3 - x1)
        a12 = 2 * (y3 - y1)
        a13 = 2 * (z3 - z1)
        a21 = 2 * (x3 - x2)
        a22 = 2 * (y3 - y2)
        a23 = 2 * (z3 - z2)
        b1 = r1 ** 2 - r3 ** 2 - x1 ** 2 - y1 ** 2 - z1 ** 2 + x3 ** 2 + y3 ** 2 + z3 ** 2
        b2 = r2 ** 2 - r3 ** 2 - x2 ** 2 - y2 ** 2 - z2 ** 2 + x3 ** 2 + y3 ** 2 + z3 ** 2

        # Check for singularities a13 = 0 and a23 = 0
        if a13 == 0 or a23 == 0:
            # substitutions for x and y coordinates
            a = 2 * (x3 - x1)
            b = 2 * (y3 - y1)
            c = r1 ** 2 - r3 ** 2 - x1 ** 2 - y1 ** 2 + x3 ** 2 + y3 ** 2
            d = 2 * (x3 - x2)
            e = 2 * (y3 - y2)
            f = r2 ** 2 - r3 ** 2 - x2 ** 2 - y2 ** 2 + x3 ** 2 + y3 ** 2

            # y and x calculations
            x = (c * e - b * f) / (a * e - b * d)
            y = (a * f - c * d) / (a * e - b * d)

            # substitutions for z coordinate
            B = -2 * z1
            C = z1 ** 2 - r1 ** 2 + (x - x1) ** 2 + (y - y1) ** 2

            z1 = (-B - np.sqrt(B ** 2 - 4 * C)) / 2
            # z2 = (-B + np.sqrt(B ** 2 - 4 * C)) / 2    not needed, as the Z coordinate can only be negative

            P = [round(x, 2), round(y, 2), round(z1, 2)]

        else:
            # Second substitutions
            a1 = a11 / a13 - a21 / a23
            a2 = a22 / a23 - a12 / a13
            a3 = b1 / a13 - b2 / a23
            a4 = a2 / a1
            a5 = a3 / a1
            a6 = (-a21 * a4 - a22) / a23
            a7 = (b2 - a21 * a5) / a23

            # Third substitutions
            a = a4 ** 2 + a6 ** 2 + 1
            b = 2 * a4 * (a5 - x1) + 2 * a6 * (a7 - z1) - 2 * y1
            c = a5 * (a5 - 2 * x1) + a7 * (a7 - 2 * z1) + x1 ** 2 + y1 ** 2 + z1 ** 2 - r1 ** 2

            if b ** 2 - 4 * a * c < 0:
                print("Provided angles are not correct. Check for Joint angle sensing error.")
                raise TypeError

            y1 = (-b + np.sqrt(b ** 2 - (4 * a * c))) / (2 * a)  # positive y coordinate
            y2 = (-b - np.sqrt(b ** 2 - (4 * a * c))) / (2 * a)

            z1 = a6 * y1 + a7
            z2 = a6 * y2 + a7

            if z1 < 0:
                y = y1
            else:
                y = y2
            P = [round(a4 * y + a5, 2), round(y, 2), round(a6 * y + a7, 2)]

        return P
