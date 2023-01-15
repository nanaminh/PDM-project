import numpy as np
from scipy import optimize
from minimum_snap import minimum_snap
from smooth_trajectory import getConstrainMtx

n_order = 7

waypoint = [[0.4, 0.4, 1], [0.8, 0.8, 1], [1.2, 0.4, 1], [1.5, 0, 1], [1.8, 0.4, 1]]
mini = minimum_snap(waypoint)

x_init = np.ones((n_order + 1) * (len(waypoint) - 1)) * 3  # create an initial guess with ones

"--- set the constraint in a way to be put in a solver ---"


def const(x):
    A_mat, b_mat = getConstrainMtx(waypoints=waypoint, n_order=7, dim='z')
    print(A_mat)
    print(b_mat)
    print("x", x)
    # print("b", np.shape(b_mat.flatten()))
    # print("returned",A_mat@x.T - b_mat.flatten())
    # print("returned shape", np.shape(A_mat@x.T - b_mat.flatten()))
    return A_mat @ x.T - b_mat.flatten()


con1 = {'type': 'eq', 'fun': const}

"--- Run optimization solver ---"
res = optimize.minimize(fun=mini.costFunc, x0=x_init, method='BFGS', constraints=con1)

print("result", res.x)
