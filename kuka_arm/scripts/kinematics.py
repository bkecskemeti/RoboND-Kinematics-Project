
def angle(a, b, c):
    return acos((a*a + b*b - c*c) / (2*a*b))

def norm(a, b):
    return sqrt(a*a + b*b)

def rotation_x(r): # roll
    return Matrix([[      1,      0,      0],
                   [      0, cos(r), -sin(r)],
                   [      0, sin(r),  cos(r)]])

def rotation_y(p): # pitch
    return Matrix([[ cos(p),      0,  sin(p)],
                   [      0,      1,       0],
                   [-sin(p),      0,  cos(p)]])

def rotation_z(y): # yaw
    return Matrix([[ cos(y),-sin(y),       0],
                   [ sin(y), cos(y),       0],
                   [      0,      0,       1]])

def transformation_matrix(alpha, a, d, q):
    return Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])
