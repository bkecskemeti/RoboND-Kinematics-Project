## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[rviz_screenshot]: ./misc_images/rviz_screenshot_1.png
[DH_model]: ./misc_images/DH_model.png
[gazebo_screenshot]: ./misc_images/gazebo_screenshot.png
[kuka_side_view]: ./misc_images/kuka_side_view.png
[R3_6_formula]: ./misc_images/R3_6_formula.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Forward kinematics rendering the Kuka KR210 robot arm with transparency and axes shown:

![Kuka KR210 robot arm][rviz_screenshot]

The kinematic model of the Kuka KR210 robot arm is detailed in the session ["Kuka KR210 Forward Kinematics Part 01"](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/398f60b1-b112-41ec-b8bc-59e95a2cec70):

![DH model][DH_model]

Regarding the model, the following can be obtained from the file [kr210.urdf.xacro](./kuka_arm/urdf/kr210.urdf.xacro):

* Link<sub>1</sub>: &alpha;<sub>0</sub>=0 (Z<sub>0</sub> || Z<sub>1</sub>), a<sub>0</sub>=0 (Z<sub>0</sub> and Z<sub>1</sub> are coincident), d<sub>1</sub> = 0.75 (distance of X<sub>0</sub> to X<sub>1</sub> along Z<sub>0</sub>)

* Link<sub>2</sub>: &alpha;<sub>1</sub> = -90&deg; (rotate Z<sub>1</sub> by 90&deg; clockwise to get Z<sub>2</sub>), a<sub>1</sub> = 0.35 (distance of Z<sub>1</sub> to Z<sub>2</sub> along X<sub>1</sub>), d<sub>2</sub> = 0 (distance of X<sub>1</sub> and X<sub>2</sub>). Note: &theta;<sub>2</sub> needs an offset of -90&deg;.

* Link<sub>3</sub>: &alpha;<sub>2</sub>=0 (Z<sub>2</sub> || Z<sub>3</sub>), a<sub>2</sub> = 1.25 (distance of Z<sub>2</sub> to Z<sub>3</sub> along X<sub>2</sub>), d<sub>3</sub> = 0 (distance of X<sub>2</sub> and X<sub>3</sub> along Z<sub>2</sub>)

* Link<sub>4</sub>: &alpha;<sub>3</sub> = -90&deg; (rotate Z<sub>3</sub> by 90&deg; clockwise to get Z<sub>4</sub>), a<sub>3</sub> = -0.054 (distance of Z<sub>3</sub> to Z<sub>4</sub> along X<sub>3</sub>), d<sub>4</sub> = 1.5 (distance of X<sub>3</sub> to X<sub>4</sub>)

* Link<sub>5</sub>: &alpha;<sub>4</sub> = 90&deg; (rotate Z<sub>4</sub> by 90&deg; counter-clockwise to get Z<sub>5</sub>), a<sub>4</sub> = 0 (Z<sub>4</sub> and Z<sub>5</sub> have same origin), d<sub>5</sub> = 0 (X<sub>4</sub> = X<sub>5</sub>)

* Link<sub>6</sub>: &alpha;<sub>5</sub> = -90&deg; (rotate Z<sub>5</sub> by 90&deg; clockwise to get Z<sub>6</sub>), a<sub>5</sub> = 0 (Z<sub>5</sub> and Z<sub>6</sub> have same origin),  d<sub>6</sub> = 0 (X<sub>5</sub> = X<sub>6</sub>)

* End Effector (EE): &alpha;<sub>EE</sub>=0 (Z<sub>6</sub> || Z<sub>G</sub>), a<sub>EE</sub> = 0 (Z<sub>6</sub> = Z<sub>G</sub>), d<sub>EE</sub> =  0.303 (distance of X<sub>6</sub> to X<sub>G</sub> along Z<sub>G</sub>)


**This is summarized in the following DH table:**

i | &alpha;<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | &theta;<sub>i</sub>
--- | --- | --- | --- | ---
1 | 0&deg; | 0 | 0.75 | &theta;<sub>1</sub>
2 | -90&deg; | 0.35 | 0 | -90&deg; + &theta;<sub>2</sub>
3 | 0&deg; | 1.25 | 0 | &theta;<sub>3</sub>
4 | -90&deg; | -0.054 | 1.5 | &theta;<sub>4</sub>
5 | 90&deg; | 0 | 0 | &theta;<sub>5</sub>
6 | -90&deg; | 0 | 0 | &theta;<sub>6</sub>
EE | 0&deg; | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Based on the description in ["Lesson 11/17 Forward Kinematics"](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/c0837700-3de6-4c41-8a5d-1e25936e0cdb), the transformation matrix is defined in [kinematics.py](./kuka_arm/scripts/kinematics.py#L29) as:

```python
def transformation_matrix(alpha, a, d, q):
    return Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])
```

Subsequently, the individual transformation matrices are calculated by simply substituting each row from the DH table:

```python
T(i-1)_i = transformation_matrix(alpha(i-1), a(i-1), d(i), q(i)).subs(DH_Table)
```

```python
T0_1 = Matrix([[  cos(theta1), -sin(theta1),   0,     0],
               [  sin(theta1),  cos(theta1),   0,     0],
               [            0,             0,  1,  0.75],
               [            0,             0,  0,     1]])

# note there is a -90 deg offset in theta2
T1_2 = Matrix([[ sin(theta2),  cos(theta2),   0,   0.35],
               [           0,            0,   1,      0],
               [ cos(theta2), -sin(theta2),   0,      0],
               [           0,            0,   0,      1]])

T2_3 = Matrix([[ cos(theta3), -sin(theta3),   0,   1.25],
               [ sin(theta3),  cos(theta3),   0,      0],
               [           0,            0,   1,      0],
               [           0,            0,   0,      1]])

T3_4 = Matrix([[ cos(theta4), -sin(theta4),   0, -0.054],
               [           0,            0,   1,    1.5],
               [-sin(theta4), -cos(theta4),   0,      0],
               [           0,            0,   0,      1]])

T4_5 = Matrix([[ cos(theta5), -sin(theta5),   0,      0],
               [           0,            0,  -1,      0],
               [ sin(theta5),  cos(theta5),   0,      0],
               [           0,            0,   0,      1]])

T5_6 = Matrix([[ cos(theta6), -sin(theta6),   0,      0],
               [           0,            0,   1,      0],
               [-sin(theta6), -cos(theta6),   0,      0],
               [           0,            0,   0,      1]])

T6_EE = Matrix([[ cos(thetaEE), -sin(thetaEE),   0,     0],
                [ sin(thetaEE),  cos(thetaEE),   0,     0],
                [            0,             0,   1, 0.303],
                [            0,             0,   0,     1]])
```

The rotation matrix between the base link and gripper link is calculated as the multiplication of rotation matrices of yaw, pitch, and roll, and then a correction matrix as described in the section ["Inverse Kinematics with Kuka KR210"](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0):

```python
# huge complicated matrix with cells linear expressions of:
# sin(theta_i), cos(theta_i), sin(theta_i + theta_i+1), cos(theta_i + theta_i+1)
T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
```

The correction matrix accounts for the difference between the orientation of the base frame and the gripper frame, and can be found by rotating 180 degrees about the z axis, then -90 degrees around the y axis:

```python
def rotation_base_ee(roll, pitch, yaw):
    rot_end_effector = rotation_z(yaw) * rotation_y(pitch) * rotation_x(roll)
    rot_correction = rotation_z(radians(180)) * rotation_y(radians(-90))
    return rot_end_effector * rot_correction
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The coordinates of the wrist center can be calculated as follows:

```python
WC = EE - (0.303) * ROT_EE[:,2]
```

I drawed the side-view of the robot arm schematically like this:

![side view][kuka_side_view]

To get the angles &theta;<sub>1</sub> ... &theta;<sub>3</sub>, we need to calculate a, b, c, &alpha;, &beta;, &gamma;. The angles can be found using the cosine law
acos( (a<sup>2</sup> + b<sup>2</sup> - c<sup>2</sup>) / 2ab ).

```python
def angle(a, b, c):
    """Calculate the angle opposite to side c.
    """
    return acos((a*a + b*b - c*c) / (2*a*b))

a = 1.5
b = sqrt(pow(norm(WC[0], WC[1]) - 0.35, 2) + pow(WC[2] - 0.75, 2))
c = 1.25

alpha = angle(side_b,side_c,side_a)
beta = angle(side_a,side_c,side_b)
gamma = angle(side_a,side_b,side_c)
```

**&Theta;<sub>1</sub>:** as &theta;<sub>1</sub> is the only angle moving the wrist center in the x-y plane, it follows easily that the angle can be obtained just from projecting the wrist center to the x-y plane:

```python
theta1 = atan2(WC[1], WC[0])
```

**&Theta;<sub>2</sub>:** (see drawing above)

```python
def norm(a, b):
    return sqrt(a*a + b*b)

theta2 = pi/2. - angle_a - atan2(WC[2] - 0.75, norm(WC[0], WC[1]) - 0.35)
```

**&Theta;<sub>3</sub>:** (see drawing above)

```python
# 0.036 = atan2(b, sqrt(a*a - d*d))
theta3 = pi/2. - (angle_b + 0.036)
```

**&Theta;<sub>4, 5, 6</sub>:**

The solution to the inverse orientation problem follows from:

<sub>6</sub><sup>0</sup>R = <sub>3</sub><sup>0</sup>R * <sub>6</sub><sup>3</sup>R &rarr;
<sub>6</sub><sup>3</sup>R = (<sub>3</sub><sup>0</sup>R)<sup>-1</sup> * <sub>6</sub><sup>0</sup>R

where: <sub>3</sub><sup>0</sup>R = <sub>1</sub><sup>0</sup>T * <sub>2</sub><sup>1</sup>T * <sub>3</sub><sup>2</sup>T with the transformation matrices already calculated above.

Calculating the multiplications and inverse:

```python
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R3_6 = simplify(R0_3.inv('LU') * ROT_EE)
```

The result is:

![Formula for R3_6][R3_6_formula]

Calculating the angles from the rotation matrix can be done as described in the section ["Euler angles"](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/a124f98b-1ed5-45f5-b8eb-6c40958c1a6b#):

```python
theta5 = atan2(norm(R3_6[0,2], R3_6[2,2]), R3_6[1,2])
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

*Note:* if `R3_6[1,2]` is approximately 0, `atan2` can "flicker" between -&pi;/2 and &pi;/2, resulting in unnecessary flapping motion of the wrist. To fix this, instead of the simple calculation above, I used the `euler_from_matrix` function from the [tf library](http://docs.ros.org/jade/api/tf/html/python/transformations.html). The source of `euler_from_matrix` can be found [here](https://www.lfd.uci.edu/~gohlke/code/transformations.py.html).

```python
def inverse_kinematics_2(R3_6):
    alpha, beta, gamma = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), axes='rxyz')
    return (np.pi/2 + theta4, np.pi/2 - theta5, gamma)
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

The code `IK_server.py` calls the logic described previously, which is implemented in `kinematics.py`.

The main methods are `inverse_kinematics_1(WC)` and `inverse_kinematics_2(R3_6)`.

The code succeeds in picking correctly the objects and dropping them in the cylinder most of the time:

![Gazebo sceenshot of dropping to cylinder][gazebo_screenshot]
