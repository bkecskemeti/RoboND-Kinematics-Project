import rospy
from sympy import *
from time import time
from mpmath import radians
import tf

def angle(a, b, c):
    """Calculate the angle opposite to side c.
    """
    return acos((a*a + b*b - c*c) / (2*a*b))

def norm(a, b):
    return sqrt(a*a + b*b)

def rotation_x(angle):
    return Matrix([[          1,          0,           0],
                   [          0, cos(angle), -sin(angle)],
                   [          0, sin(angle),  cos(angle)]])

def rotation_y(angle):
    return Matrix([[ cos(angle),          0, sin(angle)],
                   [          0,          1,          0],
                   [-sin(angle),          0, cos(angle)]])

def rotation_z(angle):
    return Matrix([[ cos(angle),-sin(angle),          0],
                   [ sin(angle), cos(angle),          0],
                   [          0,          0,          1]])

def transformation_matrix(alpha, a, d, q):
    return Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

def rotation_base_ee(roll, pitch, yaw):
    rot_end_effector = rotation_z(yaw) * rotation_y(pitch) * rotation_x(roll)
    rot_correction = rotation_z(radians(180)) * rotation_y(radians(-90))
    return rot_end_effector * rot_correction

def inverse_kinematics_1(WC):
    theta1 = atan2(WC[1], WC[0])

    side_a = 1.5
    side_b = sqrt(pow(norm(WC[0], WC[1]) - 0.35, 2) + pow(WC[2] - 0.75, 2))
    side_c = 1.25

    angle_a = angle(side_b,side_c,side_a)
    angle_b = angle(side_a,side_c,side_b)
    angle_c = angle(side_a,side_b,side_c)

    theta2 = pi/2. - angle_a - atan2(WC[2] - 0.75, norm(WC[0], WC[1]) - 0.35)
    theta3 = pi/2. - (angle_b + 0.036)

    return (theta1, theta2, theta3)

def inverse_kinematics_2(R3_6):
    theta5 = atan2(norm(R3_6[0,2], R3_6[2,2]), R3_6[1,2])

    # prevent wrist flaps
    log_branch = False
    if sin(theta5) < 0.:
        theta4 = atan2(-R3_6[2,2], R3_6[0,2])
        theta6 = atan2(R3_6[1,1], -R3_6[1,0])
        log_branch = True
    else:
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    rospy.loginfo("%s branch: sin(theta5)=%04.8f, cos(theta5)=%04.8f, theta4=%04.8f, theta5= %04.8f, theta6=%04.8f" % (log_branch, sin(theta5), cos(theta5), theta4, theta5, theta6))

    return (theta4, theta5, theta6)
