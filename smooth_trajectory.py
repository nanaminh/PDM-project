#######
# RO47005 Planning and decision making 22/23
# Group:10
# Aurthor: Danning Zhao
# email: D.ZHAO-3@student.tudelft.nl
#######

import numpy as np
import matplotlib as mpl
#import matplotlib.pyplot as plt
#import pybullet_data as pd
#import pybullet as p
import math

def getCoeff(k,n_order,ts):
    """
    Args:
        k (int): the order of derivative
        n_order (int): order of polynomial
        ts (0~1): time

    Returns:
        list: dim(1,n_order+1)
    """
    coeff=[0]*(n_order+1)
    if k==0:
        for i in range(0,n_order+1):
            coeff[i]=ts**i
    else:
        for i in range(k,n_order+1):#order number
            coeff[i]=np.math.factorial(i)/np.math.factorial(i-k)*ts**(i-k)
    
    return coeff

def getConstrainMtx(waypoints,n_order, dim):
    waypoints= np.array(waypoints)
    if len(waypoints)<2:
        raise ValueError("The waypoint number should not be less than 2!!")
    start_pos= waypoints[:-1] # Start positions of segments
    goal_pos= waypoints[1:]
    segment=len(waypoints)-1 #Number of segments]
    coordinates = {'x':0, 'y': 1, 'z':2}

    """
    return the constraint matrix, for single coordinate
    Args:
        start_pos (list): start point of each segment dim(1,n)
        goal_pos (list): goal point of each segment dim(1,n)
        n_order (int): order of polynomial
        dim (char): x, y or z
    
    Returns:
        np.array: the constraint matrix
    """
    # if segment!=segment:
    #     raise ValueError("The dimension of input start and goal list not same")
   
    
    mtxA = np.zeros((segment*(n_order+1),segment*(n_order+1)))
    mtxb = np.zeros(((n_order+1)*segment, 1)) # segment -> number of the segment, n_order -> order of the polynominal

    #1. waypoints constraint (row 0-7 when len(waypoints) is 5)
    #p i (S i−1 ) = w i−1 and p i (S i ) = w i for all i = 1, . . . , n (2*len(segment) constraints)
    for i in range(0,segment):#index number
        ##mtxA[k,k*(n_order+1):(k+1)*(n_order+1)]=getCoeff(0,n_order,0)
        ##mtxA[k+segment,k*(n_order+1):(k+1)*(n_order+1)]=getCoeff(0,n_order,1)
        mtxA[i, (n_order+1)*i+1] = 1 # first coefficient matches the start points
        mtxA[segment+i, (n_order+1)*i+1:(n_order+1)*(i+1)] = 1 # coefficients match the end points

        # print(type(coordinates[dim]))
        # print(coordinates[dim])
        mtxb[i,0] = start_pos[i, coordinates[dim]]
        mtxb[i+(segment), 0] = goal_pos[i, coordinates[dim]]  #This for loop can be combined with the for loop above
        # print(i)
        # print(segment+i)

    #2.for start and goal, the derivative 1,2,3=0 (row 8-13 when len(waypoints) is 5)
    #p 1 (S 0 ) = p (k)n (S n ) = 0 for all k = 1, . . . , 3 (6 constraints)
    for k in range(1,4):#derivative 1,2,3
        # print(np.shape(mtxA[2*segment+k,0:7]))
        # print(np.shape(mtxA[2*segment+3+k,-(n_order+1):]))
        #print(np.shape(getCoeff(n_order,k,0)))
        #print(np.shape(getCoeff(n_order,k,1)))
        mtxA[2*segment+k-1,0:n_order+1] = getCoeff(k,n_order,0) #start
        mtxA[2*segment+3+k-1,-(n_order+1):] = getCoeff(k,n_order,1)  #goal
        #print(2*segment+k)
        #print(2*segment+3+k)

    #3. continuity
    #p i (S i ) = p i+1 (S i ) = 0 for all k = 1, . . . , 6 (6*len(segment) − 6 constraints)
    for n in range(2,segment+1):
        for k in range(1,3):#1-6
            #print("n",n)
            #print("k",k)
            #mtxA[2*segment+6+(n-2)*6+k, (n-2)*(n_order+1)+1:(n*(n_order+1))] = [getCoeff(k,n_order,1),-np.array(getCoeff(k,n_order,0))]#error:bad operator - for list
            #mtxA[2*segment+6+ 6*(n-1)+k-1, (n-1)*(n_order+1):n*(n_order+1)] = getCoeff(k,n_order,1) #error:bad operator - for list
            mtxA[2*segment+6+(n-2)*6+k-1, (n-2)*(n_order+1):((n-1)*(n_order+1))] = getCoeff(k,n_order,1)#error:bad operator - for list
            mtxA[2*segment+6+(n-2)*6+k-1, (n-1)*(n_order+1):(n*(n_order+1))]=-np.array(getCoeff(k,n_order,0))
            #mtxA[2*segment+6+(n-2)*6+k-1, (n-1)*(n_order+1):(n*(n_order+1))]=-np.array(getCoeff(k,n_order,0))
            #mtxA[2*segment+6+ 6*(n-1)+k-1, n*(n_order)+k+1] = -1 # to equate from the other side
            #print(2*segment+6+ 6*(n-1)+k)
    ############################for Bmtx########################################
    #This part is moved above to marge two for loop!!
    
    print("constraint shape",np.shape(mtxA),np.shape(mtxb))
    return mtxA,mtxb

def setTime(waypoints):
    """
    Args:
        waypoints (list): a list of 3D positions

    Returns:
        T(list): accumulated time
    """
    length=[]
    waypoints=np.array(waypoints)
    for index in range(len(waypoints)-1):
        length.append(math.sqrt(sum((waypoints[index+1,:]-waypoints[index,:])**2)))
        
    T=1.5*np.array(length) #3 param is hard coded
    S=np.cumsum(T)
    return T,S

def generateTargetPos(waypoints,control_freq_hz):
    """a function which wraps all functions together

    Args:
        waypoints (list): dim(nx3),3d waypoints
        control_freq_hz (int): 

    Returns:
        target position(list): dim(3,)
        num:number of points 
    """
    target=[]
    num=0
    segment=len(waypoints)-1
    
    waypointx=np.array(waypoints)[:,0]
    waypointy=np.array(waypoints)[:,1]
    waypointz=np.array(waypoints)[:,2]
    
    mtxAx,mtxbx=getConstrainMtx(waypointx,7)
    param_x=np.linalg.inv(mtxAx)@mtxbx.T
    mtxAy,mtxby=getConstrainMtx(waypointy,7)
    param_y=np.linalg.inv(mtxAy)@mtxby.T
    mtxAz,mtxbz=getConstrainMtx(waypointz,7)
    param_z=np.linalg.inv(mtxAz)@mtxbz.T

    midpointx=[]
    midpointy=[]
    midpointz=[]
    
    T,S=setTime(waypoints)
    #S.insert(0,0)#head insert
    S=np.insert(S,0,0)
    print(T,S)
    
    delta_t=1/control_freq_hz
    for i in range(0,segment):
        for t in np.arange(S[i],S[i+1]+delta_t,delta_t):
            time=(t-S[i])/T[i]#rescale to 0-1
            #print(time)
            midpointx.append(float(getCoeff(0,7,time)@param_x[i*8:(i+1)*8]))
            midpointy.append(float(getCoeff(0,7,time)@param_y[i*8:(i+1)*8]))
            midpointz.append(float(getCoeff(0,7,time)@param_z[i*8:(i+1)*8]))
            
    target=np.array([[midpointx[i],midpointy[i],midpointz[i]] for i in range(0,len(midpointx))])
    num=len(target)
    # PERIOD=t_ttl
    # NUM_WP = int(np.ceil(control_freq_hz*PERIOD))
    # #print("sub traj",NUM_WP)
    # TARGET_POS = np.zeros((NUM_WP,3))    
    
#     waypoints=[[midpointx[i],midpointy[i],1] for i in range(0,len(midpointx))]
    # for wp in range(0,len(waypoints)-1):
    #     p.addUserDebugLine(waypoints[wp], waypoints[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    
    
    
    return target,num
    

if __name__ == "__main__":
     ########################################################################

    waypoint=[[0.4,0.4,1],[0.8,0.8,1],[1.2,0.4,1],[1.5,0,1],[1.8,0.4,1]]
    # print(setTime(waypoints))
    #target,num=generateTargetPos(waypoint,control_freq_hz=48)
    #print(num)

    #print(getConstrainMtx(waypoints= waypoint, n_order =7, dim='x'))
    A, b = getConstrainMtx(waypoints= waypoint, n_order =7, dim='x')
    print(np.shape(A))
    A_new=A[[not np.all(A[i] == 0) for i in range(A.shape[0])], :]
    print(np.shape(A_new))
    b_new=b[:np.shape(A_new)[0]]
    print(np.shape(b_new))
    # print(A[0:2, :])
    # print(b[0:2, :])
    # print(np.linalg.det(A))
    # alpha = np.linalg.solve(A[0:2, 0:], b[0:2, :]) #This does not work because A is a singular matrix

    


    # p.connect(p.GUI)
    # p.setAdditionalSearchPath(pd.getDataPath())
    # # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
    # # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # p.loadURDF("plane.urdf")
    # #p.addUserDebugLine(waypoint[0],waypoint[1],lineColorRGB=[1, 0, 0],lifeTime=0, lineWidth=1)
    # for wp in range(0,len(waypoint)-1):
    #     p.addUserDebugLine(waypoint[wp], waypoint[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    #     #print(waypoint[wp])
    # ###############################visualisation need some time#############################################
    # # for wp in range(0,len(target)-1):
    # #     p.addUserDebugLine(target[wp], target[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    #     #print(waypoint[wp])
    # #################################FASTER Visualisation#################################################################
    # for wp in range(0,len(target)-10,10):
    #     p.addUserDebugLine(target[wp], target[wp+10], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    
    ########################################################################
    #list inserting test
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
#print(parameters)
#print(np.linalg.solve(mtxA,mtxb.T))

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
    # param_x=np.linalg.inv(mtxAx)@mtxbx.T#ERROR:singular matrix, there is somethin gwrong with the matrix defined

    # mtxAy,mtxby=getConstrainMtx(y1,y2,7)
    # param_y=np.linalg.inv(mtxAy)@mtxby.T
    # # plt.plot(X, Y, color="blue", linewidth=2.5, linestyle="-")
    # # plt.show()
    # print(np.shape(param_y[0:8]))
    # midpointx=[]
    # midpointy=[]
    # step=0.1

    # for seg in range(0,segment):
    #     for i in np.arange(0,1+step,step):
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
    #DEBUG:Find the fourth floor is all zeros, there is something wrong with index
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
#    #DEBUG : to np.array 
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
#     param_x=np.linalg.inv(mtxAx)@mtxbx.T#

#     mtxAy,mtxby=getConstrainMtx(waypointy,7)
#     param_y=np.linalg.inv(mtxAy)@mtxby.T
#     # plt.plot(X, Y, color="blue", linewidth=2.5, linestyle="-")
#     # plt.show()
#     print(param_y)
#     midpointx=[]
#     midpointy=[]
#     step=0.1

#     for seg in range(0,segment):
#         for i in np.arange(0,1+step,step):
#             midpointx.append(float(getCoeff(0,7,i)@param_x[seg*8:(seg+1)*8]))
#             #print(i)
#             midpointy.append(float(getCoeff(0,7,i)@param_y[seg*8:(seg+1)*8]))
    
#     waypoints=[[midpointx[i],midpointy[i],1] for i in range(0,len(midpointx))]
#     for wp in range(0,len(waypoints)-1):
#         p.addUserDebugLine(waypoints[wp], waypoints[wp+1], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    
    ############################################################################################

