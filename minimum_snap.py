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
from scipy.optimize import minimize
from scipy.linalg import block_diag

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
    
    def getmtxQ(self):
        T,S=self.setTime()
        S=np.insert(S,0,0)
        
        # Q=[]
        # for t in T:
        #     coeff=np.reshape(np.array(self.getCoeff(4,7,t)),(8,1))
        #     tempQ=coeff@coeff.T
        #     Q=block_diag(Q,tempQ)
        # print(np.shape(Q))
        # print(Q)### PROBLEM([33,32])???
        
        #####TO PREVENT DIMENSION PROBLEM############
        coeff0=np.reshape(np.array(self.getCoeff(4,7,T[1])),(8,1))
        Q=coeff0@coeff0.T
        for t in T[1:]:
            coeff=np.reshape(np.array(self.getCoeff(4,7,t)),(8,1))
            tempQ=coeff@coeff.T
            Q=block_diag(Q,tempQ)
        print(np.shape(Q))
        
        return Q
    
    def generateTargetPos(waypoints,control_freq_hz):
        return 0
    
if __name__ == "__main__":
     ########################################################################
    
    waypoint=[[0.4,0.4,1],[0.8,0.8,1],[1.2,0.4,1],[1.5,0,1],[1.8,0.4,1]]
    # print(setTime(waypoints))
    mini=minimum_snap(waypoint)
    Q=mini.getmtxQ()
