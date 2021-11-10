import numpy as np
import VerticesCoordinates
from collisionException import *

# angle limits used in limiting the movement
OUT_OF_RANGE_ANGLE_LOW = -0.5235987756
OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268
ELBOW_JOINT_WIDTH = 45
LINK_OFFSET = ELBOW_JOINT_WIDTH / 2


class DeltaRobot():

    def __init__(self):

        self.vert_coords = VerticesCoordinates.VerticesCoordinates()
        # geometry of the robot
        self.sb = 375  # 25mm is substracted from 400 because the axes are mounted with 25mm offset  # 567  # length of the Side of the Base triangle in mm (sb)
        self.sp = 76  # length of the Side of the effector (Plate) triangle in mm (sp)
        self.Length = 200  # 524  # length of the biceps in mm (L)
        self.length = 281  # 1244  # length of the forearm in mm (l)
        self.wb = 1 / 3 * np.sqrt(3) / 2 * self.sb  # distance from the center of the Base to the side of the triangle
        self.ub = 2 * self.wb  # distance from the center of the Base to the side of the triangle
        self.wp = 1 / 3 * np.sqrt(3) / 2 * self.sp  # distance from the center of the effector (Plate) to the vertex of the triangle
        self.up = 2 * self.wp
        self.a = self.wb - self.up
        self.b = self.sp / 2 - np.sqrt(3) / 2 * self.wb
        self.c = self.wp - 1 / 2 * self.wb
        self.TCP = [0, 0, 20]  # offset for the TCP

        # safe areas - legs
        self.safe_leg_1 = [-0.866 * self.ub - 20, -self.wb - 20, -0.866 * self.ub + 20, -self.wb + 20]
        self.safe_leg_2 = [0.866 * self.ub - 20, -self.wb - 20, 0.866 * self.ub + 20, -self.wb + 20]
        self.safe_leg_3 = [-20, self.ub - 20, 20, self.ub + 20]

        # variables
        self.fi = [0, 0, 0]

        # vectors for elbow joints Ai
        self.A1 = [0.0, 0.0, 0.0]
        self.A2 = [0.0, 0.0, 0.0]
        self.A3 = [0.0, 0.0, 0.0]

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

    def calculateIPK(self, xyz):
        """Calculate inverse kinematics based on input coordinates. Input is a tuple (x,y,z)"""
        x = xyz[0]
        y = xyz[1]
        z = xyz[2]

        # If z>0 then it's not a viable configuration of robot
        if z > 0:
            print("The coordinates are out of range!")
            raise TypeError

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
                # print(f"{self.fi[i]*180/3.14= }")

        except RuntimeWarning:
            return

        else:
            ##### FOR ANIMATIONS #####

            # coordinates of elbow joints in the base frame {B} [x,y,z]
            self.A1 = [0,
                       -self.wb - self.Length * np.cos(self.fi[0]),
                       -self.Length * np.sin(self.fi[0])]
            self.A2 = [np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(self.fi[1])),
                       1 / 2 * (self.wb + self.Length * np.cos(self.fi[1])),
                       -self.Length * np.sin(self.fi[1])]
            self.A3 = [-np.sqrt(3) / 2 * (self.wb + self.Length * np.cos(self.fi[2])),
                       1 / 2 * (self.wb + self.Length * np.cos(self.fi[2])),
                       -self.Length * np.sin(self.fi[2])]

            self.createLinksVerticesLists(x, y, z)

            self.validatePoints()

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

        if z1 == z3 or z1 == z2 and not z1 == z2 == z3:
            z1 += 0.01
            #print(f"{z1 = }, {z2 = }, {z3 = }")

        if z2 == z3 and not z1 == z2 == z3:
            z2 += 0.01
            #print(f"{z1 = }, {z2 = }, {z3 = }")

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
        if a13 == 0 and a23 == 0:
            print("singularities")
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

            point = [x, y, z1]

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
            # z2 = a6 * y2 + a7

            if z1 < 0:
                y = y1
            else:
                y = y2
            point = [a4 * y + a5, y, a6 * y + a7]

        return point

    def createLinksVerticesLists(self, x, y, z):

        self.link11_A = [self.A1[0] - LINK_OFFSET, self.A1[1], self.A1[2]]
        self.link12_A = [self.A1[0] + LINK_OFFSET, self.A1[1], self.A1[2]]
        self.link21_A = [self.A2[0] + LINK_OFFSET / 2, self.A2[1] - LINK_OFFSET * np.sqrt(3) / 2, self.A2[2]]
        self.link22_A = [self.A2[0] - LINK_OFFSET / 2, self.A2[1] + LINK_OFFSET * np.sqrt(3) / 2, self.A2[2]]
        self.link31_A = [self.A3[0] + LINK_OFFSET / 2, self.A3[1] + LINK_OFFSET * np.sqrt(3) / 2, self.A3[2]]
        self.link32_A = [self.A3[0] - LINK_OFFSET / 2, self.A3[1] - LINK_OFFSET * np.sqrt(3) / 2, self.A3[2]]

        self.link11_P = [self.P1[0] + x - LINK_OFFSET, self.P1[1] + y, self.P1[2] + z]
        self.link12_P = [self.P1[0] + x + LINK_OFFSET, self.P1[1] + y, self.P1[2] + z]
        self.link21_P = [self.P2[0] + LINK_OFFSET / 2 + x, self.P2[1] - LINK_OFFSET * np.sqrt(3) / 2 + y, self.P2[2] + z]
        self.link22_P = [self.P2[0] - LINK_OFFSET / 2 + x, self.P2[1] + LINK_OFFSET * np.sqrt(3) / 2 + y, self.P2[2] + z]
        self.link31_P = [self.P3[0] + LINK_OFFSET / 2 + x, self.P3[1] + LINK_OFFSET * np.sqrt(3) / 2 + y, self.P3[2] + z]
        self.link32_P = [self.P3[0] - LINK_OFFSET / 2 + x, self.P3[1] - LINK_OFFSET * np.sqrt(3) / 2 + y, self.P3[2] + z]

        self.vert_coords.X0 = [self.B1[0], self.A1[0]]  # , self.P1[0] + x                                  link 1
        self.vert_coords.X1 = [self.link11_A[0], self.link12_A[0], self.link12_P[0], self.link11_P[0], self.link11_A[0]]     #  link 1.1 and 1.2
        self.vert_coords.X2 = [self.B2[0], self.A2[0]]  # , self.P2[0] + x                                  link 2
        self.vert_coords.X3 = [self.link21_A[0], self.link22_A[0], self.link22_P[0], self.link21_P[0], self.link21_A[0]]  #     link 2.1 and 2.2
        self.vert_coords.X4 = [self.B3[0], self.A3[0]]  # , self.P3[0] + x                                  link 3
        self.vert_coords.X5 = [self.link31_A[0], self.link32_A[0], self.link32_P[0], self.link31_P[0], self.link31_A[0]]  #     link 3.1 and 3.2
        self.vert_coords.X6 = [self.P1[0] + x, self.P2[0] + x, self.P3[0] + x, self.P1[0] + x]  #           effector

        self.vert_coords.Y0 = [self.B1[1], self.A1[1]]  # , self.P1[1] + y
        self.vert_coords.Y1 = [self.link11_A[1], self.link12_A[1], self.link12_P[1], self.link11_P[1], self.link11_A[1]]
        self.vert_coords.Y2 = [self.B2[1], self.A2[1]]  # , self.P2[1] + y
        self.vert_coords.Y3 = [self.link21_A[1], self.link22_A[1], self.link22_P[1], self.link21_P[1], self.link21_A[1]]
        self.vert_coords.Y4 = [self.B3[1], self.A3[1]]  # , self.P3[1] + y
        self.vert_coords.Y5 = [self.link31_A[1], self.link32_A[1], self.link32_P[1], self.link31_P[1], self.link31_A[1]]
        self.vert_coords.Y6 = [self.P1[1] + y, self.P2[1] + y, self.P3[1] + y, self.P1[1] + y]

        self.vert_coords.Z0 = [self.B1[2], self.A1[2]]  # , self.P1[2] + z
        self.vert_coords.Z1 = [self.link11_A[2], self.link12_A[2], self.link12_P[2], self.link11_P[2], self.link11_A[2]]
        self.vert_coords.Z2 = [self.B2[2], self.A2[2]]  # , self.P2[2] + z
        self.vert_coords.Z3 = [self.link21_A[2], self.link22_A[2], self.link22_P[2], self.link21_P[2], self.link21_A[2]]
        self.vert_coords.Z4 = [self.B3[2], self.A3[2]]  # , self.P3[2] + z
        self.vert_coords.Z5 = [self.link31_A[2], self.link32_A[2], self.link32_P[2], self.link31_P[2], self.link31_A[2]]
        self.vert_coords.Z6 = [self.P1[2] + z, self.P2[2] + z, self.P3[2] + z, self.P1[2] + z]

        self.writeVerticesToStructure(x, y, z)

    def writeVerticesToStructure(self, x, y, z):
        # list of lists of coordinates, X_coord_list_combined has all the X coordinates needed to plot the delta model: motor joint X, elbow X, effector joint X - for all links
        self.vert_coords.coordinates_x = [self.vert_coords.X0, self.vert_coords.X1,
                                          self.vert_coords.X2, self.vert_coords.X3,
                                          self.vert_coords.X4, self.vert_coords.X5,
                                          self.vert_coords.X6, x + self.TCP[0]]
        self.vert_coords.coordinates_y = [self.vert_coords.Y0, self.vert_coords.Y1,
                                          self.vert_coords.Y2, self.vert_coords.Y3,
                                          self.vert_coords.Y4, self.vert_coords.Y5,
                                          self.vert_coords.Y6, y + self.TCP[1]]
        self.vert_coords.coordinates_z = [self.vert_coords.Z0, self.vert_coords.Z1,
                                          self.vert_coords.Z2, self.vert_coords.Z3,
                                          self.vert_coords.Z4, self.vert_coords.Z5,
                                          self.vert_coords.Z6, z - self.TCP[2]]

    def validatePoints(self):
        """Check if the links are interefering with safe areas"""
        # Check for safe areas (legs in this case)
        self.SafeLegArea(self.link11_A, self.link11_P, self.safe_leg_1)
        self.SafeLegArea(self.link12_A, self.link12_P, self.safe_leg_2)
        self.SafeLegArea(self.link21_A, self.link21_P, self.safe_leg_2)
        self.SafeLegArea(self.link22_A, self.link22_P, self.safe_leg_3)
        self.SafeLegArea(self.link31_A, self.link31_P, self.safe_leg_3)
        self.SafeLegArea(self.link32_A, self.link32_P, self.safe_leg_1)

    def SafeLegArea(self, A, P, *args):
        """ Mainly to protect legs from collisions, the area is represented as an infinite height cuboid with square as a base. Args structure is: [x1,y1,x2,y2], where p1 and p2 are points on the opposite sides of the diagonal """

        for safe_object in args:
            x1 = safe_object[0]
            y1 = safe_object[1]
            x2 = safe_object[2]
            y2 = safe_object[3]

            # Swap if needed
            if y1 > y2:
                y1, y2 = y2, y1
            if x1 > x2:
                x1, x2 = x2, x1

            # Line perpendicular to y axis
            if A[0] - P[0] == 0 and y1 <= A[1] <= y2:
                if self.checkIfPointInArea([x1,A[1]], A, P) or self.checkIfPointInArea([x2, A[1]], A, P):
                    print("A[0] - P[0] == 0 and y1 <= A[1] <= y2")
                    raise CollisionException

            elif A[0] - P[0] != 0:
                a = (A[1] - P[1]) / (A[0] - P[0])
                b = P[1] - a * P[0]

                # Calculate intersection points
                x = [(y1 - b) / a, (y2 - b) / a]
                y = [a * x1 + b, a * x2 + b]

                if x1 <= x[0] <= x2:
                    if self.checkIfPointInArea([x[0], y1], A, P):
                        print("x1 <= x[0] <= x2")
                        raise CollisionException
                if x1 <= x[1] <= x2:
                    if self.checkIfPointInArea([x[1], y2], A, P):
                        print("x1 <= x[1] <= x2")
                        raise CollisionException
                if y1 <= y[0] <= y2:
                    if self.checkIfPointInArea([x1, y[0]], A, P):
                        print("y1 <= y[0] <= y2")
                        raise CollisionException
                if y1 <= y[1] <= y2:
                    if self.checkIfPointInArea([x2, y[1]], A, P):
                        print("y1 <= y[1] <= y2")
                        raise CollisionException

        return 0

    def checkIfPointInArea(self, point, A, P):
        """ Check if the collision point is within the range of the arms """
        x1 = A[0]
        x2 = P[0]
        y1 = A[1]
        y2 = P[1]

        if y1 > y2:
            y1, y2 = y2, y1
        if x1 > x2:
            x1, x2 = x2, x1

        if x1 <= point[0] <= x2 and y1 <= point[1] <= y2:
            return True

        return False

    # def safeLegArea(self, A, P, C):
    #     """ A is a list of x,y,z coordinates for the elbow joint; P is a list of x,y,z coordinates for the effector joint; C is a list of x,y,z coordinates of the center of ellipsoid and its principal semi-axes a,b,c """
    #     ### ONLY TEMPORARY """
    #     a = 20 / 2
    #     b = 20 / 2
    #     c = 1800 / 2
    #     print(A)
    #     print(P)
    #     print(C)
    #
    #
    #     # First substitutions
    #     x0 = A[0]
    #     y0 = A[1]
    #     z0 = A[2]
    #     xc = C[0]
    #     yc = C[1]
    #     zc = C[2]
    #
    #     u1 = A[0] - P[0]  # for now its only for link 1 + there have to be 2 lines for each link, offset by the length of the joint lengths
    #     u2 = A[1] - P[1]
    #     u3 = A[2] - P[2]
    #
    #     # Second substitutions
    #     m1 = y0 + yc
    #     m2 = z0 - zc
    #     m3 = u2 / u1
    #     m4 = u3 / u1
    #     m5 = (-2 * xc + xc ** 2) / (a ** 2) + (m3 ** 2 * x0 ** 2 + m1 ** 2 - 2 * m1 * m3 * x0) / (b ** 2) + (m4 ** 2 * x0 ** 2 + m2 ** 2 - 2 * m2 * m4 * x0) / (c ** 2)
    #     m6 = 1 / (a ** 2) + (m3 ** 2) / (b ** 2) + (m4 ** 2) / (c ** 2)
    #     m7 = (m1 * m3 - 2 * m3 ** 2 * x0) / (b ** 2) + (2 * m2 * m4 - 2 * m4 ** 2 * x0) / (c ** 2)
    #
    #     # Calculations
    #     if (m7 ** 2 - 4 * m6 * (m5 - 1)) >= 0 :
    #         x_pos = (m7 + np.sqrt(m7 ** 2 - 4 * m6 * (m5 - 1))) / (2 * m6)
    #         y_pos = m3 * (x_pos - x0) + y0
    #         z_pos = m4 * (x_pos - x0) + z0
    #
    #         x_neg = (m7 + np.sqrt(m7 ** 2 - 4 * m6 * (m5 - 1))) / (2 * m6)
    #         Y_neg = m3 * (x_neg - x0) + y0
    #         z_neg = m4 * (x_neg - x0) + z0
    #
    #         print("There is a problem")
    #
    #     else:
    #         print("no problem")
    #         return 1  # there is no problem with the movement
