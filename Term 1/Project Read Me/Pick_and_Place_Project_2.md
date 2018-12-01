# Problem Statement
In this project one has to edit a python script to program a Robotic arm (kuka_arm package) so that it successfully pick and place the object.

# Kinematic Analysis
To compute forward kinematics of the robotic arm, one first has to find the modified DH parameters. The urdf file(kr210.urdf.xacro) in the urdf folder provides the information regarding the angle and position between the various joints of the robot. These values will help to determine DH matrix. It is to be noted that reference frame on the joint is independent of the frame we choose for our DH matrix. To make the DH matrix, there is a certain set of rules to followed which are described in the lectures.

First, we initialze the symbols for the DH matrix.

[![N|Solid](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/5935b661_l01-16-l-denavit-hartenberg-parameter-definitions-01/l01-16-l-denavit-hartenberg-parameter-definitions-01.png)]()

$$\alpha_{i-1}$$ or twist angle : Angle between $$z_{i-1}$$ and $$z_{i}$$ about the $$x_{i-1}$$ axis
$$a_{i-1}$$ or link offset: Distance between $$z_{i-1}$$ and $$z_{i}$$ along the $$x_{i-1}$$ axis
$$d_{i}$$ or link length: Distance between $$x_{i-1}$$ and $$x_{i}$$ along the $$z_{i}$$ axis
$$\theta_{i}$$ or joint angle: Angle between $$x_{i-1}$$ and $$x_{i}$$ about the $$z_{i}$$ axis

DH Matrix symbols initilization:
```py
alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
```

The following image shows the frame assigned to calculate the DH parameter matrix.

[![N|Solid](https://www.dropbox.com/s/2zs2g7lxo5lagqu/IMG_20180624_023014.jpg?raw=1)]()

Next, we fill the values of the parameters in the DH matrix using the values in the urdf file. The urdf file is located in the inside ./kuka_arm/urdf/kr210.urdf.xacro. Lets calculate the DH parameters, the vertical distance(Z axis) betweem joint 1 and base link is 0.33 in urdf file. Joint 2 is 0.35 (X direction) and 0.42(Z direction) away from joint 1. As there is a difference in frame assigned in DH matrix and urdf file, we have to use the values in urdf file to calculate the DH paramter matrix. As $$z_{0}$$ and $$z_{1}$$ coincide therefore $$\alpha_{0}$$ and $$a_{0}$$ is zero measured along $$x_{0}$$ direction. But $$x_{1}$$ and $$x_{0}$$ are parallel and the perpendicular distance between them is 0.75 (Distance from base to joint1 + Distance from joint1 to joint2), therefore $$d_{1}$$ measured along $$z_{1}$$ is 0.75.
Similarly other values for DH parameters can be found using the urdf file and the image I have shown below, where I have marked the relevant distance either between consecutive parallel $$x_{i-1}, x_{i}$$ and $$z_{i-1}, z_{i}$$ axis. For angles q2 is little trickier, but if we follow the rules i.e angles between $$x_{i}$$ and $$x_{i-1}$$ measured along $$z_{i}$$, we can see that angle between $$x_{2}$$ and $$x_{1}$$ measured along $$z_{2}$$ is displaced by -$$90^{o}$$.  

[![N|Solid](https://www.dropbox.com/s/bx4plsj251w68b1/IMG_20180626_023647.jpg?raw=1)]()


```py
subs_dict = {   alpha0:0. , a0:0 , d1: 0.75, q1:q1,
                alpha1:-pi/2. , a1: 0.35, d2:0, q2: q2-pi/2.,
                alpha2: 0. , a2: 1.25, d3:0, q3:q3,
                alpha3: -pi/2 ,  a3: -0.054 , d4: 1.5, q4:q4, 
                alpha4:  pi/2. , a4:0 , d5:0, q5:q5,
                alpha5: -pi/2. , a5:0, d6:0, q6:q6,
                alpha6: 0. , a6: 0. , d7: 0.303, q7: 0}
```
Once we have DH matrix we now can obtain the transformation matrix between consecutive reference frames. The general transformation matrix $$^{i-1}_{i}T$$ is given as

[![N|Solid](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/593eebcd_eq1/eq1.png)]()

where c and s stands for $$\cos$$ and $$\sin$$ of an angle respectively.

To perform this computation I defined a function which takes $$\alpha_{i-1}, a_{i-1}, d_{i}, \theta_{i}$$ as input arguments and returns the symbolic transformation matrix $$^{i-1}_{i}T$$.  

```py
def TF_matrix(alpha,a,d,q):
    TF = Matrix([[cos(q),-sin(q), 0, a],
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                 [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha),cos(alpha)*d],
                 [   0,  0,  0,  1.]])
    return TF
```

While calling the transformation matrix funcation I also substituted the DH parameters values from the (subs_dict)dictionary we assigned earlier.
```py
T_01 = TF_matrix(alpha0,a0,d1,q1).subs(subs_dict)
T_12 = TF_matrix(alpha1,a1,d2,q2).subs(subs_dict)
T_23 = TF_matrix(alpha2,a2,d3,q3).subs(subs_dict)
T_34 = TF_matrix(alpha3,a3,d4,q4).subs(subs_dict)
T_45 = TF_matrix(alpha4,a4,d5,q5).subs(subs_dict)
T_56 = TF_matrix(alpha5,a5,d6,q6).subs(subs_dict)
T_6G = TF_matrix(alpha6,a6,d7,q7).subs(subs_dict)
```
*Note: G stand for Gripper not Ground.

[![N|Solid](https://www.dropbox.com/s/9qugqj55shnu5zl/Trans.PNG?raw=1)]()

After we get all the transformation matrix we have to multiply them all to obtain transformation matrix which transforms Gripper to ground frame. The transformation and the generalized matrix are listed as:


$$^{0}_{G}T$$ = $$^{0}_{1}T^ {1}_{2}T^{2}_{3}T^{3}_{4}T^{4}_{5}T^{5}_{6}T^{6}_{G}T$$
```py
T0_G = lambdify([q1,q2,q3,q4,q5,q6],T_01*T_12*T_23*T_34*T_45*T_56*T_6G)
```
I have used lambdify here , because it more faster than the usual evalf.(subs = {}) as per sympy - [Numeric Computation], where comparison between the two is shown.

The URDF model does not follow the DH convention i.e. the frames from your DH parameter table will not always match the default orientation of the KUKA arm in RViz and/or gazebo. Therefore to align them we need to multiply our current $$^{0}_{G}T$$ matrix with the Correction Rotational matrix.

After we obtain the correctional rotation matrix, we need to next calculate your end-effector pose with respect to the base link. We use the x-y-z intrinsic rotation convention. First, we rotate the gripper frame w.r.t z axis(yaw) by 180 degres, then w.r.t the new y axis by -90 degrees. The resulting rotational matrix using this convention to transform from one fixed frame to another, would be:

```py
r,p,y = symbols('r,p,y')
rot_yaw = lambdify(y,Matrix([[cos(y),-sin(y),0.],
                             [sin(y),cos(y), 0.],
                             [0.      ,0.      , 1.]]))
rot_roll = lambdify(r,Matrix([ [1.     ,0.      ,0.  ],
                               [0.      ,cos(r)  , -sin(r)],
                               [0.      ,sin(r)  , cos(r)]]))

rot_pitch = lambdify(p,Matrix([[cos(p),0.      , sin(p)],
                               [0.      ,1.0     , 0.],
                               [-sin(p) ,0.      , cos(p)]]))

rot_error = np.matrix(rot_yaw(pi))*np.matrix(rot_pitch(-pi/2.))
```
I have initialized individual rotation matrix for yaw, pitch, and roll rotations.

[![N|Solid](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/5975d719_fk/fk.png)]()

The figure above shows the urdf frame position, if one compares them with the frame of the DH parameters one can see the difference.

# Inverse Kinematics
To determine the joint angle give the pose of the robotic arm, we have to do inverse kinematics. We have divided the problem of inverse kinematics in to, inverse orientation and inverse position.

However, only certain types of manipulators are solvable in closed-form. If either of the following two conditions are satisfied, then the serial manipulator is solvable in closed-form.

1. Three neighboring joint axes intersect at a single point, or
2. Three neighboring joint axes are parallel (which is technically a          special case of 1, since parallel lines intersect at infinity)

The last three joints in kuka arm manipulator are revolute joints that satisfy condition 1, and intersect at a common point called the wrist center. The advantage of such a design is that it kinematically decouples the position and orientation of the end effector. Mathematically, this means that instead of solving twelve nonlinear equations simultaneously (one equation for each term in the first three rows of the overall homogeneous transform matrix), it is now possible to independently solve two simpler problems: first, the Cartesian coordinates of the wrist center, and then the composition of rotations to orient the end effector. The first three joints control the position of wrist center while last three orient the gripper for grasping.

We will first solve the inverse position problem, for it we need to determine Wrist Center(WC) coordinates w.r.t the ground frame. The coordinates of WC w.r.t ground can be easily obtained through the vector triangle law as shown in the figure below.

[![N|](https://d17h27t6h515a5.cloudfront.net/topher/2017/June/59375712_l20-inverse-kinematics-02/l20-inverse-kinematics-02.png)]()

We already know the end effector position w.r.t ground, and to transform the vector from  end effector to WC, we multiply it by rotation matrix $$_{6}^{0}R$$, as shown in the image below.
[![N|](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e52e6_image-4/image-4.png)]()

In the IK_server.py script we receive the yaw,pitch, and yaw angle of the gripper from simulation ROS. To account for the coorectional rotation between the end effector frame and urdf frame:
$$R_{rpy}$$ = $$R_{y}$$* $$R_{p}$$ * $$R_{r}$$*$$R_{correc/error}$$

The column vector we get are the coordinates of the wrist center w.r.t the ground.

```python
px,py,pz = req.poses[0].position.x,req.poses[0].position.y,req.poses[0].position.z
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([req.poses[0].orientation.x, req.poses[0].orientation.y, req.poses[0].orientation.z, req.poses[0].orientation.w])

Rrpy = np.matrix(rot_yaw(yaw))*np.matrix(rot_pitch(pitch))*np.matrix(rot_roll(roll)) * rot_error

r_WC = np.matrix([[px],[py],[pz]])  -  0.303*Rrpy[:,2]
```

Once we have the Wrist center coordinates, we then need find the rotation matrix $$^{3}_{0}R$$. To calculate the matrix, we need to find $$\theta1, \theta2, \theta3$$. $$\theta1$$ can be easily obtained 

To find $$\theta1$$:
[![N|](https://www.dropbox.com/s/s9vw6nkifmq1295/IMG_20180624_005451.jpg?raw=1)]()

To find $$\theta2$$:
[![N|](https://www.dropbox.com/s/gmnj1sd9mf1r7r5/IMG_20180624_005503.jpg?raw=1)]()

To find $$\theta3$$:
[![N|](https://www.dropbox.com/s/xg1961qdi1ct1kx/IMG_20180624_005518.jpg?raw=1)]()


```py
a = 1.501 
b = sqrt((sqrt(r_WC[0,0]**2 + r_WC[1,0]**2) - 0.35)**2 + (r_WC[2,0]-0.75)**2)
c = 1.25 #a2

theta1 = atan2(r_WC[1,0],r_WC[0,0])
theta2 = pi/2. - acos((b*b + c*c - a*a)/(2*b*c)) - atan2((r_WC[2,0]-0.75),(sqrt(r_WC[0,0]*r_WC[0,0] + r_WC[1,0]*r_WC[1,0]) - 0.35))
theta3 = pi/2. - acos((a*a + c*c - b*b)/(2*a*c)) - 0.036
```

Then we calculate the rotation matrix $$_{3}^{0}R$$. I have used numpy matrices as numpy matrixes are computationally faster. To solve for the remaining joint angles
$$\theta4,\theta5, \theta6$$, which we can compute if we know the $$_{6}^{3}R$$. To calculate $$_{6}^{3}R$$, we multipy inverse of $$_{3}^{0}R$$ with $$R_{rpy}$$. 

```py
R_03 = np.matrix(R_01(theta1))*np.matrix(R_12(theta2))*np.matrix(R_23(theta3))
R_36 = R_03.T * Rrpy
```
Once we get the $$_{6}^{3}R$$ matrix we can compare the individual terms to the values on R.H.S of the equation. 
```py
theta4 = atan2(R_36[2,2],-R_36[0,2])
theta5 = atan2(sqrt(R_36[0,2]**2 + R_36[2,2]**2),R_36[1,2])
theta6 = atan2(-R_36[1,1],R_36[1,0])
```   
A image of Pick and Place:
9/10 Samples placed in the box

[![N|](https://www.dropbox.com/s/3e76np8nfzcsvha/IMG_20180626_020502.jpg?raw=1)]()
   [Numeric Computation]: <http://docs.sympy.org/latest/modules/numeric-computation.html>
