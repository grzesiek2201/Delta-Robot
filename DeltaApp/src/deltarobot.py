import VerticesCoordinates
from collisionException import *
from math import sqrt, sin, cos, atan


# angle limits used in limiting the movement
OUT_OF_RANGE_ANGLE_LOW = -0.5235987756
OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268
ELBOW_JOINT_WIDTH = 45
LINK_OFFSET = ELBOW_JOINT_WIDTH / 2
SQRT_3 = sqrt(3)
SQRT_3_2 = sqrt(3) / 2

class DeltaRobot():

    def __init__(self):

        self.vert_coords = VerticesCoordinates.VerticesCoordinates()
        self.initializeCoords()
        # geometry of the robot
        self.sb = 375  # 25mm is subtracted from 400 because the axes are mounted with 25mm offset  # 567  # length of the Side of the Base triangle in mm (sb)
        self.sp = 76  # length of the Side of the effector (Plate) triangle in mm (sp)
        self.Length = 200  # 524  # length of the biceps in mm (L)
        self.length = 281  # 1244  # length of the forearm in mm (l)
        self.Length2 = self.Length ** 2  # square of Length
        self.length2 = self.length ** 2  # square of length
        self.wb = 1 / 3 * SQRT_3_2 * self.sb  # distance from the center of the Base to the side of the triangle
        self.ub = 2 * self.wb  # distance from the center of the Base to the side of the triangle
        self.wp = 1 / 3 * SQRT_3_2 * self.sp  # distance from the center of the effector (Plate) to the vertex of the triangle
        self.up = 2 * self.wp
        self.a = self.wb - self.up
        self.b = self.sp / 2 - SQRT_3_2 * self.wb
        self.c = self.wp - 1 / 2 * self.wb
        self.TCP = [0, 0, -100]  # offset for the TCP
        self.z_limit = -430

        # safe areas - legs
        self.safe_leg_1 = [-0.866 * self.ub - 25, -self.wb - 25, -0.866 * self.ub + 25, -self.wb + 25]
        self.safe_leg_2 = [0.866 * self.ub - 25, -self.wb - 25, 0.866 * self.ub + 25, -self.wb + 25]
        self.safe_leg_3 = [-25, self.ub - 25, 25, self.ub + 25]

        # variables
        self.fi = [0, 0, 0]

        # vectors for elbow joints Ai
        self.A1 = [0.0, 0.0, 0.0]
        self.A2 = [0.0, 0.0, 0.0]
        self.A3 = [0.0, 0.0, 0.0]

        # vectors for base joints Bi in base frame {B}
        self.B1 = [0, -self.wb, 0]
        self.B2 = [SQRT_3_2 * self.wb, 1 / 2 * self.wb, 0]
        self.B3 = [-SQRT_3_2 * self.wb, 1 / 2 * self.wb, 0]

        # vectors for effector joints Pi in effector frame {P}
        self.P1 = [0, -self.up, 0]
        self.P2 = [self.sp / 2, self.wp, 0]
        self.P3 = [-self.sp / 2, self.wp, 0]

        # coordinates of the base vertices (for animation)
        self.Bvx = [-self.sb / 2, 0, self.sb / 2, -self.sb / 2]
        self.Bvy = [-self.wb, self.ub, -self.wb, -self.wb]
        self.Bvz = [0, 0, 0, 0]

    def initializeCoords(self):
        """ Initialize vertices coordinates to 0 """
        self.vert_coords.coordinates_x =[[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
        self.vert_coords.coordinates_y =[[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
        self.vert_coords.coordinates_z =[[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]

    def calculateIPK(self, xyz):
        """ Calculate inverse kinematics based on input coordinates. Input is a tuple (x,y,z) """
        x = xyz[0] - self.TCP[0]
        y = xyz[1] - self.TCP[1]
        z = xyz[2] - self.TCP[2]

        # If z>-70 then it's not a viable configuration of robot
        if z + self.TCP[2] > -70 or z + self.TCP[2] < self.z_limit:
            print("The coordinates are out of range!")
            raise TypeError

        # constants used to calculate Inverse Kinematics
        Ei, Fi, Gi = self.calculateConstants((x, y, z))

        # try to calculate the joint angles, if they are out of limits, throw an OutOfRangeError
        try:
            for i in range(0, 3):
                checkForRealSolutions((Ei, Fi, Gi), index=i)
                feg_sqrt = sqrt(Fi[i] ** 2 + Ei[i] ** 2 - Gi[i] ** 2)
                temp = 2 * atan((-Fi[i] + feg_sqrt) / (Gi[i] - Ei[i]))
                if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                    self.fi[i] = temp
                else:
                    temp = 2 * atan((-Fi[i] - feg_sqrt) / (Gi[i] - Ei[i]))
                    if OUT_OF_RANGE_ANGLE_LOW <= temp <= OUT_OF_RANGE_ANGLE_HIGH:
                        self.fi[i] = temp
                    else:
                        print("The coordinates are out of range!")
                        raise TypeError

        except RuntimeWarning:
            return

        else:
            ##### FOR ANIMATIONS #####
            # coordinates of elbow joints in the base frame {B} [x,y,z]
            self.calculateAMatrix()

            self.createLinksVerticesLists(x, y, z)

            self.validatePoints()

    def calculateFPK(self, fi, radians=False):
        """ Calculate forward kinematics based on supplied angles fi """
        L = self.Length
        # Convert degrees to radians
        if not radians:
            fi1, fi2, fi3 = degToRadians(fi)
        else:
            fi1, fi2, fi3 = fi
        # Calculate xi, yi, zi based on fi1, fi2, fi3
        x1 = 0
        y1 = -self.wb - L * cos(fi1) + self.up
        z1 = -L * sin(fi1)

        x2 = SQRT_3_2 * (self.wb + L * cos(fi2)) - self.sp / 2
        y2 = 1 / 2 * (self.wb + L * cos(fi2)) - self.wp
        z2 = -L * sin(fi2)

        x3 = -SQRT_3_2 * (self.wb + L * cos(fi3)) + self.sp / 2
        y3 = 1 / 2 * (self.wb + L * cos(fi3)) - self.wp
        z3 = -L * sin(fi3)

        r1, r2, r3 = self.length, self.length, self.length  # all three radia are the same length because the arms are symmetric
        r12 = r1 ** 2
        r22 = r2 ** 2
        r32 = r3 ** 2
        x12 = x1 ** 2
        x22 = x2 ** 2
        x32 = x3 ** 2
        y12 = y1 ** 2
        y22 = y2 ** 2
        y32 = y3 ** 2
        z12 = z1 ** 2
        z22 = z2 ** 2
        z32 = z3 ** 2
        # to get rid of computation problems with dividing by zero
        if z1 == z3 or z1 == z2 and not z1 == z2 == z3:
            z1 += 0.01

        if z2 == z3 and not z1 == z2 == z3:
            z2 += 0.01

        # First substitutions
        a11 = 2 * (x3 - x1)
        a12 = 2 * (y3 - y1)
        a13 = 2 * (z3 - z1)
        a21 = 2 * (x3 - x2)
        a22 = 2 * (y3 - y2)
        a23 = 2 * (z3 - z2)
        b1 = r12 - r32 - x12 - y12 - z12 + x32 + y32 + z32
        b2 = r22 - r32 - x22 - y22 - z22 + x32 + y32 + z32

        # Check for singularities a13 = 0 and a23 = 0
        if a13 == 0 and a23 == 0:
            print("singularities")
            # substitutions for x and y coordinates
            a = 2 * (x3 - x1)
            b = 2 * (y3 - y1)
            c = r12 - r32 - x12 - y12 + x32 + y32
            d = 2 * (x3 - x2)
            e = 2 * (y3 - y2)
            f = r22 - r32 - x22 - y22 + x32 + y32

            # y and x calculations
            x = (c * e - b * f) / (a * e - b * d)
            y = (a * f - c * d) / (a * e - b * d)

            # substitutions for z coordinate
            B = -2 * z1
            C = z12 - r12 + (x - x1) ** 2 + (y - y1) ** 2

            z1 = (-B - sqrt(B ** 2 - 4 * C)) / 2
            # z2 = (-B + sqrt(B ** 2 - 4 * C)) / 2    not needed, as the Z coordinate can only be negative

            point = [x + self.TCP[0], y + self.TCP[1], z1 + self.TCP[2]]

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
            c = a5 * (a5 - 2 * x1) + a7 * (a7 - 2 * z1) + x12 + y12 + z12 - r12

            if b ** 2 - 4 * a * c < 0:
                print("Provided angles are not correct. Check for Joint angle sensing error.")
                raise TypeError

            y_sqrt = sqrt(b ** 2 - (4 * a * c))
            y1 = (-b + y_sqrt) / (2 * a)  # positive y coordinate
            y2 = (-b - y_sqrt) / (2 * a)

            z1 = a6 * y1 + a7
            # z2 = a6 * y2 + a7

            if z1 < 0:
                y = y1
            else:
                y = y2
            point = [a4 * y + a5 + self.TCP[0], y + self.TCP[1], a6 * y + a7 + self.TCP[2]]

        return point

    def calculateConstants(self, xyz):
        """ Calculate and return Ei Fi Gi constants """
        L = self.Length
        l2 = self.length2
        L2 = self.Length2
        x, y, z = xyz
        x2 = x ** 2
        y2 = y ** 2
        z2 = z ** 2
        a2 = self.a ** 2
        b2 = self.b ** 2
        c2 = self.c ** 2
        Ei = [2 * L * (y + self.a),
              -L * (SQRT_3 * (x + self.b) + y + self.c),
              L * (SQRT_3 * (x - self.b) - y - self.c)]
        Fi = [2 * z * L, 2 * z * L, 2 * z * L]
        Gi = [x2 + y2 + z2 + a2 + L2 + 2 * y * self.a - l2,
              x2 + y2 + z2 + b2 + c2 + L2 + 2 * x * self.b + 2 * y * self.c - l2,
              x2 + y2 + z2 + b2 + c2 + L2 - 2 * x * self.b + 2 * y * self.c - l2]
        return Ei, Fi, Gi

    def calculateAMatrix(self):
        L = self.Length
        self.A1 = [0,
                   -self.wb - L * cos(self.fi[0]),
                   -L * sin(self.fi[0])]
        self.A2 = [SQRT_3_2 * (self.wb + L * cos(self.fi[1])),
                   1 / 2 * (self.wb + L * cos(self.fi[1])),
                   -L * sin(self.fi[1])]
        self.A3 = [-SQRT_3_2 * (self.wb + L * cos(self.fi[2])),
                   1 / 2 * (self.wb + L * cos(self.fi[2])),
                   -L * sin(self.fi[2])]

    def createLinksVerticesLists(self, x, y, z):

        self.link11_A = [self.A1[0] - LINK_OFFSET, self.A1[1], self.A1[2]]
        self.link12_A = [self.A1[0] + LINK_OFFSET, self.A1[1], self.A1[2]]
        self.link21_A = [self.A2[0] + LINK_OFFSET / 2, self.A2[1] - LINK_OFFSET * SQRT_3_2, self.A2[2]]
        self.link22_A = [self.A2[0] - LINK_OFFSET / 2, self.A2[1] + LINK_OFFSET * SQRT_3_2, self.A2[2]]
        self.link31_A = [self.A3[0] + LINK_OFFSET / 2, self.A3[1] + LINK_OFFSET * SQRT_3_2, self.A3[2]]
        self.link32_A = [self.A3[0] - LINK_OFFSET / 2, self.A3[1] - LINK_OFFSET * SQRT_3_2, self.A3[2]]

        self.link11_P = [self.P1[0] + x - LINK_OFFSET, self.P1[1] + y, self.P1[2] + z]
        self.link12_P = [self.P1[0] + x + LINK_OFFSET, self.P1[1] + y, self.P1[2] + z]
        self.link21_P = [self.P2[0] + x + LINK_OFFSET / 2, self.P2[1] - LINK_OFFSET * SQRT_3_2 + y, self.P2[2] + z]
        self.link22_P = [self.P2[0] + x - LINK_OFFSET / 2, self.P2[1] + LINK_OFFSET * SQRT_3_2 + y, self.P2[2] + z]
        self.link31_P = [self.P3[0] + x + LINK_OFFSET / 2, self.P3[1] + LINK_OFFSET * SQRT_3_2 + y, self.P3[2] + z]
        self.link32_P = [self.P3[0] + x - LINK_OFFSET / 2, self.P3[1] - LINK_OFFSET * SQRT_3_2 + y, self.P3[2] + z]

        self.vert_coords.X0 = [self.B1[0], self.A1[0]]  # link 1
        self.vert_coords.X1 = [self.link11_A[0], self.link12_A[0], self.link12_P[0], self.link11_P[0], self.link11_A[0]] # link 1.1 and 1.2
        self.vert_coords.X2 = [self.B2[0], self.A2[0]]  # link 2
        self.vert_coords.X3 = [self.link21_A[0], self.link22_A[0], self.link22_P[0], self.link21_P[0], self.link21_A[0]] # link 2.1 and 2.2
        self.vert_coords.X4 = [self.B3[0], self.A3[0]]  # link 3
        self.vert_coords.X5 = [self.link31_A[0], self.link32_A[0], self.link32_P[0], self.link31_P[0], self.link31_A[0]] # link 3.1 and 3.2
        self.vert_coords.X6 = [self.P1[0] + x, self.P2[0] + x, self.P3[0] + x, self.P1[0] + x]  # effector

        self.vert_coords.Y0 = [self.B1[1], self.A1[1]]
        self.vert_coords.Y1 = [self.link11_A[1], self.link12_A[1], self.link12_P[1], self.link11_P[1], self.link11_A[1]]
        self.vert_coords.Y2 = [self.B2[1], self.A2[1]]
        self.vert_coords.Y3 = [self.link21_A[1], self.link22_A[1], self.link22_P[1], self.link21_P[1], self.link21_A[1]]
        self.vert_coords.Y4 = [self.B3[1], self.A3[1]]
        self.vert_coords.Y5 = [self.link31_A[1], self.link32_A[1], self.link32_P[1], self.link31_P[1], self.link31_A[1]]
        self.vert_coords.Y6 = [self.P1[1] + y, self.P2[1] + y, self.P3[1] + y, self.P1[1] + y]

        self.vert_coords.Z0 = [self.B1[2], self.A1[2]]
        self.vert_coords.Z1 = [self.link11_A[2], self.link12_A[2], self.link12_P[2], self.link11_P[2], self.link11_A[2]]
        self.vert_coords.Z2 = [self.B2[2], self.A2[2]]
        self.vert_coords.Z3 = [self.link21_A[2], self.link22_A[2], self.link22_P[2], self.link21_P[2], self.link21_A[2]]
        self.vert_coords.Z4 = [self.B3[2], self.A3[2]]
        self.vert_coords.Z5 = [self.link31_A[2], self.link32_A[2], self.link32_P[2], self.link31_P[2], self.link31_A[2]]
        self.vert_coords.Z6 = [self.P1[2] + z, self.P2[2] + z, self.P3[2] + z, self.P1[2] + z]

        self.writeVerticesToStructure(x, y, z)

    def writeVerticesToStructure(self, x, y, z):
        """ list of lists of coordinates, X_coord_list_combined has all the X coordinates needed to plot the delta model: motor joint X, elbow X, effector joint X - for all links """
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
                                          self.vert_coords.Z6, z + self.TCP[2]]

    def validatePoints(self):
        """Check if the links are interefering with safe areas"""
        # Check for safe areas (legs in this case)
        SafeLegArea(self.link11_A, self.link11_P, self.safe_leg_1)
        SafeLegArea(self.link12_A, self.link12_P, self.safe_leg_2)
        SafeLegArea(self.link21_A, self.link21_P, self.safe_leg_2)
        SafeLegArea(self.link22_A, self.link22_P, self.safe_leg_3)
        SafeLegArea(self.link31_A, self.link31_P, self.safe_leg_3)
        SafeLegArea(self.link32_A, self.link32_P, self.safe_leg_1)


def degToRadians(fi):
    """ Convert degrees to radians """
    fi1, fi2, fi3 = fi
    fi1 = fi1 * 3.14 / 180
    fi2 = fi2 * 3.14 / 180
    fi3 = fi3 * 3.14 / 180

    return fi1, fi2, fi3


def SafeLegArea(A, P, *args):
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
            if checkIfPointInArea([x1,A[1]], A, P) or checkIfPointInArea([x2, A[1]], A, P):
                print("A[0] - P[0] == 0 and y1 <= A[1] <= y2")
                raise CollisionException

        elif A[0] - P[0] != 0:
            a = (A[1] - P[1]) / (A[0] - P[0])
            b = P[1] - a * P[0]

            # Calculate intersection points
            x = [(y1 - b) / a, (y2 - b) / a]
            y = [a * x1 + b, a * x2 + b]

            if x1 <= x[0] <= x2:
                if checkIfPointInArea([x[0], y1], A, P):
                    print("x1 <= x[0] <= x2")
                    raise CollisionException
            if x1 <= x[1] <= x2:
                if checkIfPointInArea([x[1], y2], A, P):
                    print("x1 <= x[1] <= x2")
                    raise CollisionException
            if y1 <= y[0] <= y2:
                if checkIfPointInArea([x1, y[0]], A, P):
                    print("y1 <= y[0] <= y2")
                    raise CollisionException
            if y1 <= y[1] <= y2:
                if checkIfPointInArea([x2, y[1]], A, P):
                    print("y1 <= y[1] <= y2")
                    raise CollisionException

    return 0


def checkForRealSolutions(constants, index):
    """Checks if there are any real solutions"""
    Ei, Fi, Gi = constants
    if Fi[index] ** 2 + Ei[index] ** 2 - Gi[index] ** 2 < 0:
        raise TypeError


def checkIfPointInArea(point, A, P):
    """ Check if the collision point is within the range of the arms """
    x1 = A[0]
    x2 = P[0]
    y1 = A[1]
    y2 = P[1]

    if y1 > y2:
        y1, y2 = y2, y1
    if x1 > x2:
        x1, x2 = x2, x1

    return x1 <= point[0] <= x2 and y1 <= point[1] <= y2

