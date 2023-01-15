import numpy as np
from minimum_snap import minimum_snap
from smooth_trajectory_original import getConstrainMtx
import cvxopt

n_order = 7

waypoints = [[0.4, 0.4, 1], [0.8, 0.8, 1], [1.2, 0.4, 1], [1.5, 0, 1], [1.8, 0.4, 1]]
waypoints = [[0.4, 0.4, 1], [0.8, 0.8, 1], [1.2, 0.4, 1]]
# waypoints=[[0.4,0.4,1],[0.8,0.8,1],[1.2,0.4,1],[1.5,0,1],[1.5,0,2]]
waypoints = [[0, 0, 0], [-1, -1, 1], [1, 2, 1.5], [3, 1, 2]]
mini = minimum_snap(waypoints=waypoints)
waypointx = np.array(waypoints)[:, 0]
waypointy = np.array(waypoints)[:, 1]
waypointz = np.array(waypoints)[:, 2]

mtxAx, mtxbx = getConstrainMtx(waypointx, 7, 3)
# param_x=np.linalg.inv(mtxAx)@mtxbx.T
mtxAy, mtxby = getConstrainMtx(waypointy, 7, 4)
# param_y=np.linalg.inv(mtxAy)@mtxby.T
mtxAz, mtxbz = getConstrainMtx(waypointz, 7, 4)
# param_z=np.linalg.inv(mtxAz)@mtxbz.T


# print(mtxAx)
# print(mtxbx)
# print(mtxAx.shape)
# print(mtxbx.shape)

Amat = mtxAx[[not np.all(mtxAx[i] == 0) for i in range(mtxAx.shape[0])], :]
print(np.shape(Amat))
print(mtxbx.shape)
mtxbx = mtxbx.T
bmat = mtxbx[:np.shape(Amat)[0]]
print(np.shape(bmat))
Q = mini.getmtxQ()
print(Q.shape)
#################ERROR###################
# Amat=mtxAx
# bmat=mtxbx.T
# print(np.shape(bmat))
# Traceback (most recent call last):
#   File "try_run_optimization.py", line 47, in <module>
#     sol=cvxopt.solvers.qp(Q,q, A=A, b=b)
#   File "/home/danningzhao/anaconda3/envs/drones/lib/python3.8/site-packages/cvxopt/coneprog.py", line 4485, in qp
#     return coneqp(P, q, G, h, None, A,  b, initvals, kktsolver = kktsolver, options = options)
#   File "/home/danningzhao/anaconda3/envs/drones/lib/python3.8/site-packages/cvxopt/coneprog.py", line 2013, in coneqp
#     raise ValueError("Rank(A) < p or Rank([P; A; G]) < n")
# ValueError: Rank(A) < p or Rank([P; A; G]) < n
# ################solver##################
Q = cvxopt.matrix(Q)
A = cvxopt.matrix(Amat)
b = cvxopt.matrix(bmat)
q = cvxopt.matrix(np.zeros((np.shape(Amat)[1], 1)))

sol = cvxopt.solvers.qp(Q, q, A=A, b=b)
print(sol["x"])
