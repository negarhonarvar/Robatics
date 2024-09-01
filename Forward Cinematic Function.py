"""HW1-Q3"""
import math

import numpy as np
# the implementation of this def is based on lecture 2 pages 12-30
def find_Current_Position(x,y,theta,v1,v2,t,l):

 # we need to move the current Coordinates to robot System Coordinates
 x_R = (v1+v2)/2
 y_R = 0
 theta_R=(v1-v2)/(2*l)
 R_Matris=np.array([[np.cos((-1)*theta),np.sin((-1)*theta),0],[np.sin((-1)*theta)*(-1),np.cos((-1)*theta),0],[0,0,1]])
 kesi_dot=np.array([[x_R],[y_R],[theta_R]])
 # kesi_dot_I = [[sum(a * b for a, b in zip(R_Matris_row, kesi_dot_col))
 #           for kesi_dot_col in zip(*kesi_dot)]
 #          for R_Matris_row in R_Matris]
 kesi_dot_I=np.dot(R_Matris,kesi_dot)
 kesi_dot_I[0]=kesi_dot_I[0]*t+x
 kesi_dot_I[1]=kesi_dot_I[1]*t+y
 kesi_dot_I[2]=kesi_dot_I[2]*t+theta
 print("the final coordinates of the robot are :")
 for r in kesi_dot_I:
  print(r)


print("for the first part of the question")
find_Current_Position(1.5,2,math.pi/2,0.3,0.3,3,0.5)
print("for the second part of the question")
find_Current_Position(1.5,2,math.pi/2,0.1,-0.1,1,0.5)
print("for the last part of the question")
find_Current_Position(1.5,2,math.pi/2,0.2,0,2,0.5)
