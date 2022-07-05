import math, universal_robot_kinematics, numpy as np, random as rand, scipy
from scipy.spatial.transform import Rotation as R
rand.seed()


x,y,z = 0.3, 0.3, 0.3

o_p = [0.5, 0.5, 0]


y_hyp = math.sqrt(o_p[0]**2+o_p[2]**2)
y_rot = math.asin(o_p[0]/y_hyp)


x_hyp = math.sqrt(o_p[1]**2+o_p[2]**2)
x_rot = math.asin(o_p[1]/y_hyp) + math.pi/2

z_hyp = math.sqrt(o_p[1]**2+o_p[0]**2)
z_rot = math.asin(o_p[0]/y_hyp)

r = R.from_euler('xyz',[x_rot ,y_rot, z_rot], degrees=True)
rotmat = r.as_matrix()


print('x_rot: ', x_rot, 'y_rot: ', y_rot, 'z_rot: ', z_rot)


desired_pos = np.matrix([
[ rotmat[0][0], rotmat[0][1], rotmat[0][2], x],
[ rotmat[1][0], rotmat[1][1], rotmat[1][2], y],
[ rotmat[2][0], rotmat[2][1], rotmat[2][2], z],
[ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

links = (universal_robot_kinematics.invKine(desired_pos))[:,0]

print(links)

