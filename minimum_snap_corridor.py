import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pybullet_data as pd
import pybullet as p
import math

import cvxopt
#from smooth_trajectory_original import getConstrainMtx
from minimum_snap import minimum_snap

class minimum_snap_corridor(minimum_snap):
    def __init__(self,waypoints):
        """
        Args:
            waypoints (list): a list of 3D positions
        """
        super().__init__(waypoints)
        #self.waypoints=waypoints
    def setTime(self):
        """
        Args:
            waypoints (list): a list of 3D positions

        Returns:
            T(list): accumulated time
        """
        T,S=super().setTime()
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
        coeff=super().getCoeff(k,n_order,ts)
        return coeff
    
    def getmtxQ(self,n_order=7):
        Q=super().getmtxQ(n_order=n_order)
        return Q
    
    def costFunc(self,x):
        """cost function of minimum snap xtQx

        Args:
            x (array): parameters need to be optimized
        """
        cost=super().costFunc()
        return cost
    
    def getConstrainMtx(self,waypoints,n_order=7):
        """
        return the constraint matrix, for single coordinate
        Args:
        
            n_order (int): order of polynomial

        Returns:
            np.array: the constraint matrix
        """
        mtxA=[]
        mtxb=[]
        return mtxA,mtxb
    
    def generateTargetPos(self,control_freq_hz):
        waypoints=np.array(self.waypoints)
        target=[]
        num=0
        segment=len(waypoints)-1
        
        
        
        return target,num