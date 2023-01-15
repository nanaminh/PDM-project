#######
# RO47005 Planning and decision making 22/23
# Group:10
#######

import numpy as np
import pybullet_data as pd
import pybullet as p
from scipy.linalg import block_diag
import cvxopt
import smooth_trajectory_original


class minimum_snap(smooth_trajectory_original.smooth_trajectory):
    def __init__(self, waypoints):
        """
        Args:
            waypoints (list): a list of 3D positions
        """
        super().__init__(waypoints)
        # self.waypoints=waypoints

    def setTime(self):
        """
        Args:
            waypoints (list): a list of 3D positions

        Returns:
            T(list): accumulated time
        """
        T, S = super().setTime()
        return T, S

    def getCoeff(self, k, n_order, ts):
        """
        Args:
            k (int): the order of derivative
            n_order (int): order of polynomial
            ts (0~1): time

        Returns:
            list: dim(1,n_order+1)
        """
        coeff = super().getCoeff(k, n_order, ts)
        return coeff

    def getmtxQ(self, n_order=7):
        Q_prev = []
        n_seg = len(self.waypoints) - 1
        T, _ = self.setTime()
        # T=np.ones((len(T)))
        print("t", T)
        for k in range(0, n_seg):
            Q_k = np.zeros((n_order + 1, n_order + 1))
            for i in range(4, n_order + 1):
                for j in range(4, n_order + 1):
                    Q_k[i, j] = np.math.factorial(i) / np.math.factorial(i - 4) * np.math.factorial(
                        j) / np.math.factorial(j - 4) / (i + j - n_order) * T[k] ** (i + j - n_order)
            ############TO PREVENT CONCATENATION DIMENSION TROUBLE###############
            if Q_prev != []:
                Q_prev = block_diag(Q_prev, Q_k)
            else:
                Q_prev = Q_k
            # minimum_snap.py:100: DeprecationWarning: elementwise comparison failed; this will raise an error in the future.
            # if Q_prev!=[]:

        Q = Q_prev
        return Q

    #########################################################
    def costFunc(self, x):
        """cost function of minimum snap xtQx

        Args:
            x (array): parameters need to be optimized
        """
        Q = self.getmtxQ()
        dimension = len(Q)
        coeff = np.reshape(np.array(x), (dimension, 1))
        cost = coeff.T @ Q @ coeff
        return cost

    def getConstrainMtx(self, waypoints, n_order=7):
        """
        return the constraint matrix, for single coordinate
        Args:
            start_pos (list): start point of each sgment dim(1,n)
            goal_pos (list): goal point of each sgment dim(1,n)
            n_order (int): order of polynomial

        Returns:
            np.array: the constraint matrix
        """
        # waypoints=self.waypoints
        T, S = self.setTime()
        print("t", T)
        S = np.insert(S, 0, 0)
        if len(waypoints) < 2:
            raise ValueError("The waypoint number should not be less than 2!!")
        start_pos = waypoints[:-1]
        goal_pos = waypoints[1:]
        segment = len(start_pos)
        n_all_poly = (n_order + 1) * segment

        # if segment!=segment:
        #     raise ValueError("The dimension of input start and goal list not same")

        # mtxA=np.zeros((segment*(n_order+1),segment*(n_order+1)))
        # mtxb=np.zeros(segment*(n_order+1))
        #########################for Amtx####################################
        # 1.for start and goal, the derivative0, 1,2,3=0
        mtxA = []
        mtxb = []

        mtxA_start = np.zeros((4, n_all_poly))
        mtxb_start = np.zeros((4, 1))
        for k in range(0, 4):  # derivative 0,1,2,3
            mtxA_start[k, 0:n_order + 1] = self.getCoeff(k, n_order, 0)
        mtxb_start[0] = start_pos[0]

        # print(mtxA_start)
        # print(mtxb_start)
        # print(mtxA_start.shape)
        # print(mtxb_start.shape)
        mtxA_end = np.zeros((4, n_all_poly))
        mtxb_end = np.zeros((4, 1))
        for k in range(0, 4):  # derivative 0,1,2,3
            mtxA_end[k, -(n_order + 1):] = self.getCoeff(k, n_order, T[-1])
        mtxb_end[0] = goal_pos[-1]

        # print(mtxA_end)
        # print(mtxb_end)
        # print(mtxA_end.shape)
        # print(mtxb_end.shape)

        mtxA = np.vstack((mtxA_start, mtxA_end))
        mtxb = np.vstack((mtxb_start, mtxb_end))

        # 3.position constrain in all middle waypoints
        mtxA_middle = np.zeros((segment - 1, n_all_poly))
        mtxb_middle = np.zeros((segment - 1, 1))
        for k in range(0, segment - 1):  # index number
            mtxb_middle[k] = waypoints[k + 1]
            mtxA_middle[k, k * (n_order + 1):(k + 1) * (n_order + 1)] = self.getCoeff(0, n_order, T[k])
        #     print(waypoints[k+1])
        #     print("time",T[k])
        # #print(T)
        # print(mtxA_middle)
        # print(mtxb_middle)
        mtxA = np.vstack((mtxA, mtxA_middle))
        mtxb = np.vstack((mtxb, mtxb_middle))

        # 4.continuity constraints
        mtxA_continue = np.zeros((4 * (segment - 1), n_all_poly))
        mtxb_continue = np.zeros((4 * (segment - 1), 1))
        # print(mtxA_continue.shape)
        # print(mtxb_continue.shape)
        for n in range(1, segment):
            for k in range(0, 4):
                # print("n",n)
                # print("k",k)
                # mtxA[2*segment+6+(n-2)*6+k, (n-2)*(n_order+1)+1:(n*(n_order+1))] = [getCoeff(k,n_order,1),-np.array(getCoeff(k,n_order,0))]#error:bad operator - for list
                # print(mtxA_continue[(n-1)*4+k, (n-1)*(n_order+1):(n*(n_order+1))] )
                mtxA_continue[(n - 1) * 4 + k, (n - 1) * (n_order + 1):(n * (n_order + 1))] = self.getCoeff(k, n_order,
                                                                                                            T[n - 1])
                mtxA_continue[(n - 1) * 4 + k, n * (n_order + 1):((n + 1) * (n_order + 1))] = -np.array(
                    self.getCoeff(k, n_order, 0))
                # print("time",n,T[n-1])
        # print(mtxA_continue)
        # print(mtxb_continue)
        mtxA = np.vstack((mtxA, mtxA_continue))
        mtxb = np.vstack((mtxb, mtxb_continue))

        # print(np.shape(mtxA),np.shape(mtxb))
        return mtxA, mtxb

    def generateTargetPos(self, control_freq_hz):
        waypoints = np.array(self.waypoints)
        target = []
        num = 0
        segment = len(waypoints) - 1
        ################get waypoint of different dimension#####################
        waypointx = np.array(waypoints)[:, 0]
        waypointy = np.array(waypoints)[:, 1]
        waypointz = np.array(waypoints)[:, 2]
        #####################get constraint matrix################################
        Amatx, bmatx = self.getConstrainMtx(waypointx)
        Amaty, bmaty = self.getConstrainMtx(waypointy)
        Amatz, bmatz = self.getConstrainMtx(waypointz)
        #########################get cost function####################
        Q = self.getmtxQ()
        print(Q.shape)
        Q = cvxopt.matrix(Q)
        ###dimx
        Ax = cvxopt.matrix(Amatx)
        bx = cvxopt.matrix(bmatx)
        q = cvxopt.matrix(np.zeros((np.shape(Amatx)[1], 1)))
        solx = cvxopt.solvers.qp(Q, q, A=Ax, b=bx)
        param_x = np.array(solx["x"])
        # print(paramx)
        # rint(paramx.shape)
        # print(np.array(paramx))
        ###dimy
        Ay = cvxopt.matrix(Amaty)
        by = cvxopt.matrix(bmaty)
        q = cvxopt.matrix(np.zeros((np.shape(Amaty)[1], 1)))
        soly = cvxopt.solvers.qp(Q, q, A=Ay, b=by)
        param_y = np.array(soly["x"])
        # print(paramy)
        ###dimz
        Az = cvxopt.matrix(Amatz)
        bz = cvxopt.matrix(bmatz)
        q = cvxopt.matrix(np.zeros((np.shape(Amatz)[1], 1)))
        solz = cvxopt.solvers.qp(Q, q, A=Az, b=bz)
        param_z = np.array(solz["x"])
        # print(paramz)

        midpointx = []
        midpointy = []
        midpointz = []

        T, S = self.setTime()
        # S.insert(0,0)#head insert
        S = np.insert(S, 0, 0)
        # print(T,S)

        delta_t = 1 / control_freq_hz
        for i in range(0, segment):
            for t in np.arange(0, T[i] + delta_t, delta_t):
                # time=(t-S[i])/T[i]#rescale to 0-1
                time = t
                # print(time)
                midpointx.append(float(self.getCoeff(0, 7, time) @ param_x[i * 8:(i + 1) * 8]))
                midpointy.append(float(self.getCoeff(0, 7, time) @ param_y[i * 8:(i + 1) * 8]))
                midpointz.append(float(self.getCoeff(0, 7, time) @ param_z[i * 8:(i + 1) * 8]))

        target = np.array([[midpointx[i], midpointy[i], midpointz[i]] for i in range(0, len(midpointx))])
        num = len(target)

        return target, num


if __name__ == "__main__":
    ########################################################################

    waypoint = [[0.4, 0.4, 1], [0.8, 0.8, 1], [1.2, 0.4, 1], [1.5, 0, 1], [1.8, 0.4, 1]]
    # waypoint=[[4,0,0],[8,0,0],[12,0,0]]
    # print(setTime(waypoin
    mini = minimum_snap(waypoint)
    target, num = mini.generateTargetPos(40)
    #########################MINIMUM SNAP VISUALISATION###############################################

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.loadURDF("plane.urdf")
    # p.addUserDebugLine(waypoint[0],waypoint[1],lineColorRGB=[1, 0, 0],lifeTime=0, lineWidth=1)
    for wp in range(0, len(waypoint) - 1):
        p.addUserDebugLine(waypoint[wp], waypoint[wp + 1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
        # print(waypoint[wp])
    ###############################visualisation need some time#############################################
    for wp in range(0, len(target) - 1):
        p.addUserDebugLine(target[wp], target[wp + 1], lineColorRGB=[0, 1, 0], lifeTime=0, lineWidth=1)

    #################################FASTER Visualisation#################################################################
    # for wp in range(0,len(target)-10,10):
    #     p.addUserDebugLine(target[wp], target[wp+10], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    #########################MORE CONSTRAINTS VISUALIZATION############################################################
    smooth = smooth_trajectory_original.smooth_trajectory(waypoint)
    target, num = smooth.generateTargetPos(control_freq_hz=40)

    for wp in range(0, len(target) - 1):
        p.addUserDebugLine(target[wp], target[wp + 1], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=1)

#####################################################################
# print(Q)
# x=np.zeros((1,32))
# print(mini.costFunc(x))


######################ERROR BECAUSE OF WRONG CONSTRAINTS#################################
#     Amat,bmat=getConstrainMtx(waypointx,n_order=7,derivative=4)
#     bmat=bmat.T

# #     Traceback (most recent call last):
# #   File "minimum_snap.py", line 253, in <module>
# #     sol=cvxopt.solvers.qp(Q,q, A=A, b=b)
# #   File "/home/danningzhao/anaconda3/envs/drones/lib/python3.8/site-packages/cvxopt/coneprog.py", line 4485, in qp
# #     return coneqp(P, q, G, h, None, A,  b, initvals, kktsolver = kktsolver, options = options)
# #   File "/home/danningzhao/anaconda3/envs/drones/lib/python3.8/site-packages/cvxopt/coneprog.py", line 2013, in coneqp
# #     raise ValueError("Rank(A) < p or Rank([P; A; G]) < n")
# # ValueError: Rank(A) < p or Rank([P; A; G]) < n
###################################################################
# print(Amatx.shape)
# print(bmatx.shape)
# print(bmatx)


# Q=mini.getmtxQ()
# print(Q.shape)

# # print(Q)
# Q=cvxopt.matrix(Q)
# A=cvxopt.matrix(Amaty)
# b=cvxopt.matrix(bmaty)
# q=cvxopt.matrix(np.zeros((np.shape(Amaty)[1],1)))


# sol=cvxopt.solvers.qp(Q,q, A=A, b=b)
# print(sol["x"])
####################################################################
