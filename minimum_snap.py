#######
# RO47005 Planning and decision making 22/23
# Group:10
# Aurthor: Danning Zhao
# email: D.ZHAO-3@student.tudelft.nl
#######

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pybullet_data as pd
import pybullet as p
import math
#from scipy.optimize import minimize, LinearConstraint
from scipy.linalg import block_diag
import cvxopt

class minimum_snap:
    def __init__(self,waypoints):
        """
        Args:
            waypoints (list): a list of 3D positions
        """
        self.waypoints=waypoints
    
    def setTime(self):
        """
        Args:
            waypoints (list): a list of 3D positions

        Returns:
            T(list): accumulated time
        """
        length=[]
        waypoints=np.array(self.waypoints)
        for index in range(len(waypoints)-1):
            length.append(math.sqrt(sum((waypoints[index+1,:]-waypoints[index,:])**2)))
            
        T=1.5*np.array(length) #3 param is hard coded
        S=np.cumsum(T)
        return T,S
    
    def getCoeff(self,k,n_order,ts):
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
    ###################WRONG COST FUNCTION######################################3
    # def getmtxQ(self):
    #     T,S=self.setTime()
    #     S=np.insert(S,0,0)
        
    #     # Q=[]
    #     # for t in T:
    #     #     coeff=np.reshape(np.array(self.getCoeff(4,7,t)),(8,1))
    #     #     tempQ=coeff@coeff.T
    #     #     Q=block_diag(Q,tempQ)
    #     # print(np.shape(Q))
    #     # print(Q)### PROBLEM([33,32])???
        
    #     ####TO PREVENT DIMENSION PROBLEM############
    #     coeff0=np.reshape(np.array(self.getCoeff(4,7,T[1])),(8,1))
    #     Q=coeff0@coeff0.T
    #     for t in T[1:]:
    #         coeff=np.reshape(np.array(self.getCoeff(4,7,t)),(8,1))
    #         tempQ=coeff@coeff.T
    #         Q=block_diag(Q,tempQ)
    #     #print(np.shape(Q))
        
    #     return Q
    
   
    def getmtxQ(self,n_order=7):
        Q_prev=[]
        n_seg=len(self.waypoints)-1
        T,_=self.setTime()
        print("t",T)
        for k in range(0,n_seg):
            Q_k = np.zeros((n_order + 1, n_order + 1))
            for i in range(4,n_order+1):
                for j in range(4,n_order+1):
                    Q_k[i,j] = np.math.factorial(i)/np.math.factorial(i-4)*np.math.factorial(j)/np.math.factorial(j-4)/(i+j-n_order)*T[k]**(i+j-n_order)
            ############TO PREVENT DIMENSION TROUBLE###############
            if Q_prev!=[]:
                Q_prev = block_diag(Q_prev, Q_k)
            else:
                Q_prev=Q_k
        
        Q=Q_prev
        return Q
    
    def costFunc(self,x):
        """cost function of minimum snap xtQx

        Args:
            x (array): parameters need to be optimized
        """
        Q=self.getmtxQ()
        dimension=len(Q)
        coeff=np.reshape(np.array(x),(dimension,1))
        cost=coeff.T@Q@coeff
        return cost
    
    def getConstrainMtx(self,waypoints,n_order=7):
        """
        return the constraint matrix, for single coordinate
        Args:
            start_pos (list): start point of each sgment dim(1,n)
            goal_pos (list): goal point of each sgment dim(1,n)
            n_order (int): order of polynomial

        Returns:
            np.array: the constraint matrix
        """
        #waypoints=self.waypoints
        T,S=self.setTime()
        print("t",T)
        S=np.insert(S,0,0)
        if len(waypoints)<2:
            raise ValueError("The waypoint number should not be less than 2!!")
        start_pos=waypoints[:-1]
        goal_pos=waypoints[1:]
        segment=len(start_pos)
        n_all_poly=(n_order+1)*segment
        
        # if segment!=segment:
        #     raise ValueError("The dimension of input start and goal list not same")
    
        
        # mtxA=np.zeros((segment*(n_order+1),segment*(n_order+1)))
        # mtxb=np.zeros(segment*(n_order+1))
        #########################for Amtx####################################
        #1.for start and goal, the derivative0, 1,2,3=0
        mtxA=[]
        mtxb=[]
        
        mtxA_start = np.zeros((4, n_all_poly))
        mtxb_start = np.zeros((4, 1))
        for k in range(0,4):#derivative 0,1,2,3
            mtxA_start[k,0:n_order+1] = self.getCoeff(k,n_order,0)
        mtxb_start[0]=start_pos[0]
        
        # print(mtxA_start)
        # print(mtxb_start)
        # print(mtxA_start.shape)
        # print(mtxb_start.shape)
        mtxA_end = np.zeros((4, n_all_poly))
        mtxb_end = np.zeros((4, 1))
        for k in range(0,4):#derivative 0,1,2,3
            mtxA_end[k,-(n_order+1):] = self.getCoeff(k,n_order,T[-1])
        mtxb_end[0]=goal_pos[-1]

        # print(mtxA_end)
        # print(mtxb_end)
        # print(mtxA_end.shape)
        # print(mtxb_end.shape)
        
        mtxA=np.vstack((mtxA_start,mtxA_end))
        mtxb=np.vstack((mtxb_start,mtxb_end))
        
        #3.position constrain in all middle waypoints
        mtxA_middle = np.zeros((segment-1, n_all_poly))
        mtxb_middle = np.zeros((segment-1, 1))
        for k in range(0,segment-1):#index number
            mtxb_middle[k] = waypoints[k+1]
            mtxA_middle[k,k*(n_order+1):(k+1)*(n_order+1)]=self.getCoeff(0,n_order,T[k])
        #     print(waypoints[k+1])
        #     print("time",T[k])
        # #print(T)
        # print(mtxA_middle)
        # print(mtxb_middle)
        mtxA=np.vstack((mtxA,mtxA_middle))
        mtxb=np.vstack((mtxb,mtxb_middle))
        
        #4.continuity constraints
        mtxA_continue = np.zeros((4*(segment-1), n_all_poly))
        mtxb_continue = np.zeros((4*(segment-1), 1))
        # print(mtxA_continue.shape)
        # print(mtxb_continue.shape)
        for n in range(1,segment):
            for k in range(0,4):
                # print("n",n)
                # print("k",k)
                #mtxA[2*segment+6+(n-2)*6+k, (n-2)*(n_order+1)+1:(n*(n_order+1))] = [getCoeff(k,n_order,1),-np.array(getCoeff(k,n_order,0))]#error:bad operator - for list
                #print(mtxA_continue[(n-1)*4+k, (n-1)*(n_order+1):(n*(n_order+1))] )
                mtxA_continue[(n-1)*4+k, (n-1)*(n_order+1):(n*(n_order+1))] = self.getCoeff(k,n_order,T[n-1])
                mtxA_continue[(n-1)*4+k, n*(n_order+1):((n+1)*(n_order+1))]=-np.array(self.getCoeff(k,n_order,0))
                # print("time",n,T[n-1])
        # print(mtxA_continue)
        # print(mtxb_continue)
        mtxA=np.vstack((mtxA,mtxA_continue))
        mtxb=np.vstack((mtxb,mtxb_continue))
        
        # print(np.shape(mtxA),np.shape(mtxb))
        return mtxA,mtxb

    
    
    
    def generateTargetPos(waypoints,control_freq_hz):
        return 0
    
if __name__ == "__main__":
     ########################################################################
    
   # waypoint=[[0.4,0.4,1],[0.8,0.8,1],[1.2,0.4,1],[1.5,0,1],[1.8,0.4,1]]
    waypoint=[[4,0,0],[8,0,0],[12,0,0]]
    # print(setTime(waypoints))
    
    waypointx=np.array(waypoint)[:,0]
    mini=minimum_snap(waypoint)
    Q=mini.getmtxQ()
    print(Q.shape)
   # print(Q)
    # x=np.zeros((1,32))
    # print(mini.costFunc(x))
    Amat,bmat=mini.getConstrainMtx(waypointx)
    print(Amat.shape)
    print(bmat.shape)
    print(bmat)
    Q=mini.getmtxQ()
    print(Q.shape)
    # print(Q)
    Q=cvxopt.matrix(Q)
    A=cvxopt.matrix(Amat)
    b=cvxopt.matrix(bmat)
    q=cvxopt.matrix(np.zeros((np.shape(Amat)[1],1)))


    sol=cvxopt.solvers.qp(Q,q, A=A, b=b)
    print(sol["x"])