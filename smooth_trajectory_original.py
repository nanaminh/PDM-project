#######
# RO47005 Planning and decision-making 22/23
# Group:10
#######

import numpy as np
import pybullet_data as pd
import pybullet as p
import math


class smooth_trajectory():
    """
    a basic class to generate smooth trajectory
    """

    def __init__(self, waypoints) -> None:
        self.waypoints = waypoints

    def getCoeff(self, k, n_order, ts):
        """
        Args:
            k (int): the order of derivative
            n_order (int): order of polynomial
            ts (0~1): time

        Returns:
            list: dim(1,n_order+1)
        """
        coeff = [0] * (n_order + 1)
        if k == 0:
            for i in range(0, n_order + 1):
                coeff[i] = ts ** i
        else:
            for i in range(k, n_order + 1):  # order number
                coeff[i] = np.math.factorial(i) / np.math.factorial(i - k) * ts ** (i - k)

        return coeff

    def getConstrainMtx(self, waypoints, n_order, derivative=7):
        """
        return the constraint matrix, for single coordinate
        Args:
            waypoints: of 1 dim
            start_pos (list): start point of each sgment dim(1,n)
            goal_pos (list): goal point of each sgment dim(1,n)
            n_order (int): order of polynomial
            

        Returns:
            np.array: the constraint matrix
        """

        if len(waypoints) < 2:
            raise ValueError("The waypoint number should not be less than 2!!")
        start_pos = waypoints[:-1]
        goal_pos = waypoints[1:]
        segment = len(start_pos)

        # if segment!=segment:
        #     raise ValueError("The dimension of input start and goal list not same")

        print("integer", segment)
        mtxA = np.zeros((segment * (n_order + 1), segment * (n_order + 1)))
        mtxb = np.zeros(segment * (n_order + 1))
        #########################for Amtx####################################
        # 1. waypoints constraint
        # p i (S i−1 ) = w i−1 and p i (S i ) = w i for all i = 1, . . . , n (2*len(segment) constraints)
        for k in range(0, segment):  # index number
            mtxA[k, k * (n_order + 1):(k + 1) * (n_order + 1)] = self.getCoeff(0, n_order, 0)
            mtxA[k + segment, k * (n_order + 1):(k + 1) * (n_order + 1)] = self.getCoeff(0, n_order, 1)

        # 2.for start and goal, the derivative 1,2,3=0
        # p 1 (S 0 ) - p (k)n (S n ) = 0 for all k = 1, . . . , 3 (6 constraints)
        for k in range(1, 4):  # derivative 1,2,3
            # print(np.shape(mtxA[2*segment+k,0:7]))
            # print(np.shape(mtxA[2*segment+3+k,-(n_order+1):]))
            # print(np.shape(getCoeff(n_order,k,0)))
            # print(np.shape(getCoeff(n_order,k,1)))
            mtxA[2 * segment + k - 1, 0:n_order + 1] = self.getCoeff(k, n_order, 0)
            mtxA[2 * segment + 3 + k - 1, -(n_order + 1):] = self.getCoeff(k, n_order, 1)

        # 3. continuity
        # p i (S i ) - p i+1 (S i ) = 0 for all k = 1, . . . , 6 (6*len(segment) − 6 constraints)
        for n in range(2, segment + 1):
            for k in range(1, derivative):  ############original (1,7)1-6
                # print("n",n)
                # print("k",k)
                # mtxA[2*segment+6+(n-2)*6+k, (n-2)*(n_order+1)+1:(n*(n_order+1))] = [getCoeff(k,n_order,1),-np.array(getCoeff(k,n_order,0))]#error:bad operator - for list
                mtxA[2 * segment + 6 + (n - 2) * (derivative - 1) + k - 1,
                (n - 2) * (n_order + 1):((n - 1) * (n_order + 1))] = self.getCoeff(k, n_order,
                                                                                   1)  # error:bad operator - for list
                mtxA[2 * segment + 6 + (n - 2) * (derivative - 1) + k - 1,
                (n - 1) * (n_order + 1):(n * (n_order + 1))] = -np.array(self.getCoeff(k, n_order, 0))
                # print("coefficient",-np.array(getCoeff(k,n_order,0)))
        ############################for Bmtx########################################
        mtxb = np.zeros((1, (n_order + 1) * segment))
        for i in range(0, segment):
            mtxb[0, i] = start_pos[i]
            mtxb[0, i + (segment)] = goal_pos[i]

        # print(np.shape(mtxA),np.shape(mtxb))
        return mtxA, mtxb

    def setTime(self):
        """
        Args:
            waypoints (list): a list of 3D positions

        Returns:
            T(list): accumulated time
        """
        length = []
        waypoints = np.array(self.waypoints)
        for index in range(len(waypoints) - 1):
            length.append(math.sqrt(sum((waypoints[index + 1, :] - waypoints[index, :]) ** 2)))

        T = 1.5 * np.array(length)  # 3 param is hard coded
        S = np.cumsum(T)
        return T, S

    def generateTargetPos(self, control_freq_hz):
        """a function which wraps all functions together

        Args:
            waypoints (list): dim(nx3),3d waypoints
            control_freq_hz (int): 

        Returns:
            target position(list): dim(3,)
            num:number of points 
        """
        target = []
        num = 0
        segment = len(self.waypoints) - 1
        waypoints = np.array(self.waypoints)

        waypointx = np.array(waypoints)[:, 0]
        waypointy = np.array(waypoints)[:, 1]
        waypointz = np.array(waypoints)[:, 2]

        mtxAx, mtxbx = self.getConstrainMtx(waypointx, 7)
        param_x = np.linalg.inv(mtxAx) @ mtxbx.T
        mtxAy, mtxby = self.getConstrainMtx(waypointy, 7)
        param_y = np.linalg.inv(mtxAy) @ mtxby.T
        mtxAz, mtxbz = self.getConstrainMtx(waypointz, 7)
        param_z = np.linalg.inv(mtxAz) @ mtxbz.T

        midpointx = []
        midpointy = []
        midpointz = []

        T, S = self.setTime()
        # S.insert(0,0)#head insert
        S = np.insert(S, 0, 0)
        print(T, S)

        delta_t = 1 / control_freq_hz
        for i in range(0, segment):
            for t in np.arange(S[i], S[i + 1] + delta_t, delta_t):
                time = (t - S[i]) / T[i]  # rescale to 0-1
                # print(time)
                midpointx.append(float(self.getCoeff(0, 7, time) @ param_x[i * 8:(i + 1) * 8]))
                midpointy.append(float(self.getCoeff(0, 7, time) @ param_y[i * 8:(i + 1) * 8]))
                midpointz.append(float(self.getCoeff(0, 7, time) @ param_z[i * 8:(i + 1) * 8]))

        target = np.array([[midpointx[i], midpointy[i], midpointz[i]] for i in range(0, len(midpointx))])
        num = len(target)
        # PERIOD=t_ttl
        # NUM_WP = int(np.ceil(control_freq_hz*PERIOD))
        # #print("sub traj",NUM_WP)
        # TARGET_POS = np.zeros((NUM_WP,3))    

        #     waypoints=[[midpointx[i],midpointy[i],1] for i in range(0,len(midpointx))]
        # for wp in range(0,len(waypoints)-1):
        #     p.addUserDebugLine(waypoints[wp], waypoints[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)

        return target, num


if __name__ == "__main__":
    ########################################################################

    waypoint = [[0.4, 0.4, 1], [0.8, 0.8, 1], [1.2, 0.4, 1], [1.5, 0, 1], [1.8, 0.4, 1]]
    # print(setTime(waypoints))
    smooth = smooth_trajectory(waypoint)
    target, num = smooth.generateTargetPos(control_freq_hz=48)
    print(num)

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
    # for wp in range(0,len(target)-1):
    #     p.addUserDebugLine(target[wp], target[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    # print(waypoint[wp])
    #################################FASTER Visualisation#################################################################
    for wp in range(0, len(target) - 10, 10):
        p.addUserDebugLine(target[wp], target[wp + 10], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)

    ########################################################################
    # list inserting test
# a=[0,0,0,0]
# b=[1,1]
# a[1:3]=b
# # print(a)
# # print([0]*8)
# # print(getCoeff(1,7,1))
# # print(getCoeff(2,7,1))
# # print(getCoeff(3,7,1))
# c=np.array([0,0,0,0])
# c[1:3]=[1,1]
# #print(c[-3:])
# d1=[40,80,40,30]
# d2=[80,40,30,50]
# mtxA,mtxb=getConstrainMtx(d1,d2,7)
# parameters=np.linalg.pinv(mtxA)@mtxb.T#.inv singular
# print(parameters)
# print(np.linalg.solve(mtxA,mtxb.T))

# X=[40,80,120,80,40]
# x1=[40,80,120,80]
# x2=[80,120,80,40]
# Y=[40,80,40,0,40]
# y1=[40,80,40,0]
# y2=[80,40,0,40]
####################################################################
# X=[40,80,120,150,180]
# x1=[40,80,120,150]
# x2=[80,120,150,180]
# Y=[40,80,40,0,40]
# y1=[40,80,40,0]
# y2=[80,40,0,40]


# segment=len(x1)

# mtxAx,mtxbx=getConstrainMtx(x1,x2,7)
# param_x=np.linalg.inv(mtxAx)@mtxbx.T#ERROR:singular,说明矩阵错误

# mtxAy,mtxby=getConstrainMtx(y1,y2,7)
# param_y=np.linalg.inv(mtxAy)@mtxby.T
# # plt.plot(X, Y, color="blue", linewidth=2.5, linestyle="-")
# # plt.show()
# print(np.shape(param_y[0:8]))
# midpointx=[]
# midpointy=[]
# step=0.1

# for seg in range(0,segment):
#     for i in np.arange(0,1+step,step):#ERROR:step 较小更好,注意+step
#         midpointx.append(float(getCoeff(0,7,i)@param_x[seg*8:(seg+1)*8]))
#         #print(i)
#         midpointy.append(float(getCoeff(0,7,i)@param_y[seg*8:(seg+1)*8]))

# plt.plot(midpointx, midpointy, color="blue", linewidth=2.5, linestyle="-")
# plt.plot(X, Y)
# plt.show()
###########################################################
# x1=[40,80]
# x2=[80,120]
# mtxAx,mtxbx=getConstrainMtx(x1,x2,7)
# 进行debug,发现第4行全为0，即index出现问题
# print(np.shape(mtxAx))
# param_x=np.linalg.inv(mtxAx)@mtxbx.T
##############################trajectory generation and visualisation test###########################################


#     waypoint=[[0.4,0.4,1],[0.8,0.8,1],[1.2,0.4,1],[1.5,0,1],[1.8,0.4,1]]
#     #waypoint=[[0.4,0.4,1][0.8,0.8,1][1.2,0.4,1][1.5,0,1][1.8,0.4,1]]
#     # X=[0.4,0.8,1.2,1.5,1.8]
#     # x1=[0.4,0.8,1.2,1.5]
#     # x2=[0.8,1.2,1.5,1.8]
#     # Y=[0.4,0.8,0.4,0,0.4]
#     # y1=[0.4,0.8,0.4,0]
#     # y2=[0.8,0.4,0,0.4]
#    # print(np.shape(waypoint[:,0]))
#    #DEBUG : to np.array https://blog.csdn.net/a546167160/article/details/88398998
#     waypointx=np.array(waypoint)[:,0]
#     waypointy=np.array(waypoint)[:,1]

#     p.connect(p.GUI)
#     p.setAdditionalSearchPath(pd.getDataPath())
#     # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
#     # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#     p.loadURDF("plane.urdf")
#     #p.addUserDebugLine(waypoint[0],waypoint[1],lineColorRGB=[1, 0, 0],lifeTime=0, lineWidth=1)
#     for wp in range(0,len(waypoint)-1):
#         p.addUserDebugLine(waypoint[wp], waypoint[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
#         print(waypoint[wp])


#     segment=len(waypoint)-1
#     mtxAx,mtxbx=getConstrainMtx(waypointx,7)
#     param_x=np.linalg.inv(mtxAx)@mtxbx.T#ERROR:singular,说明矩阵错误

#     mtxAy,mtxby=getConstrainMtx(waypointy,7)
#     param_y=np.linalg.inv(mtxAy)@mtxby.T
#     # plt.plot(X, Y, color="blue", linewidth=2.5, linestyle="-")
#     # plt.show()
#     print(param_y)
#     midpointx=[]
#     midpointy=[]
#     step=0.1

#     for seg in range(0,segment):
#         for i in np.arange(0,1+step,step):#ERROR:step 较小更好,注意+step
#             midpointx.append(float(getCoeff(0,7,i)@param_x[seg*8:(seg+1)*8]))
#             #print(i)
#             midpointy.append(float(getCoeff(0,7,i)@param_y[seg*8:(seg+1)*8]))

#     waypoints=[[midpointx[i],midpointy[i],1] for i in range(0,len(midpointx))]
#     for wp in range(0,len(waypoints)-1):
#         p.addUserDebugLine(waypoints[wp], waypoints[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)

############################################################################################
