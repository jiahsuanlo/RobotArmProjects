from sympy import *

thx,thy,thz,thz1= symbols("thx thy thz thz1")

# rotation x
rotx= Matrix([[1,0,0],[0,cos(thx),-sin(thx)],[0,sin(thx),cos(thx)]])
roty= Matrix([[cos(thy),0,sin(thy)],[0,1,0],[-sin(thy),0,cos(thy)]])
rotz= Matrix([[cos(thz),-sin(thz),0],[sin(thz),cos(thz),0],[0,0,1]])
rotz1= Matrix([[cos(thz1),-sin(thz1),0],[sin(thz1),cos(thz1),0],[0,0,1]])

# rotation zyz
r_zyz= trigsimp(rotz*roty*rotz1)

# rotation zyx 
r_zyx= trigsimp(rotz*roty*rotx)
print r_zyx
 

