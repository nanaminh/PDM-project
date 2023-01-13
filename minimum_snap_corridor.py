import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pybullet_data as pd
import pybullet as p
import math

import cvxopt
from smooth_trajectory_original import smooth_trajectory
from minimum_snap import minimum_snap

class minimum_snap_corridor(minimum_snap):
    def __init__(self,waypoints):
        """
        Args:
            waypoints (list): a list of 3D positions
        """
        super().__init__(waypoints)
        self.samples=[]#samples for inequality contraints
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
    def sampling(self,number):
        """sample among each time segment
        return np.array dim()
        """
        T,_=self.setTime()
        
        samples=np.array([[time/(number+1)*(i+1) for i in range(0,number)]for time in T])
        print("T",T)
        print(samples)
        # print(samples.shape)
        return samples
    
    def getConstrainMtx(self,waypoints,n_order=7,number_sampling=4,corridor=0.7):
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

        #########################for Amtx####################################
        #1.for start and goal, the derivative0, 1,2,3=0
        mtxA=[]
        mtxb=[]
        mtxG=[]
        mtxh=[]
        
        mtxA_start = np.zeros((4, n_all_poly))
        mtxb_start = np.zeros((4, 1))
        for k in range(0,4):#derivative 0,1,2,3
            mtxA_start[k,0:n_order+1] = self.getCoeff(k,n_order,0)
        mtxb_start[0]=start_pos[0]
        mtxA_end = np.zeros((4, n_all_poly))
        mtxb_end = np.zeros((4, 1))
        
        for k in range(0,4):#derivative 0,1,2,3
            mtxA_end[k,-(n_order+1):] = self.getCoeff(k,n_order,T[-1])
        mtxb_end[0]=goal_pos[-1]
        mtxA=np.vstack((mtxA_start,mtxA_end))
        mtxb=np.vstack((mtxb_start,mtxb_end))
        
        # #3.EQUALITY position constrains in all middle waypoints
        # mtxA_middle = np.zeros((segment-1, n_all_poly))
        # mtxb_middle = np.zeros((segment-1, 1))
        # for k in range(0,segment-1):#index number
        #     mtxb_middle[k] = waypoints[k+1]
        #     mtxA_middle[k,k*(n_order+1):(k+1)*(n_order+1)]=self.getCoeff(0,n_order,T[k])
        # #     print(waypoints[k+1])
        # #     print("time",T[k])
        # # #print(T)
        # # print(mtxA_middle)
        # # print(mtxb_middle)
        # mtxA=np.vstack((mtxA,mtxA_middle))
        # mtxb=np.vstack((mtxb,mtxb_middle))
        
        ##INEQUALITY CONTSRAINTS ON MIDDLE WAYPOINTS
        #for each middle waypoint:
        #corridor=0.2#hard coded
        mtxG_middle = np.zeros((2*(segment-1), n_all_poly))
        mtxh_middle = np.zeros((2*(segment-1), 1))
        for k in range(0,segment-1):#index number
            mtxh_middle[k] = waypoints[k+1]+corridor/5
            mtxh_middle[k+segment-1] = -(waypoints[k+1]-corridor/5)
            mtxG_middle[k,k*(n_order+1):(k+1)*(n_order+1)]=self.getCoeff(0,n_order,T[k])
            mtxG_middle[k+segment-1,k*(n_order+1):(k+1)*(n_order+1)]=-np.array(self.getCoeff(0,n_order,T[k]))

        mtxG=mtxG_middle
        mtxh=mtxh_middle
        
        ################more sampling points################################
        #################2 sampling point in each segment###########################

        time=self.sampling(number=number_sampling)
        mtxG_sample = np.zeros((2*number_sampling*segment, n_all_poly))
        mtxh_sample = np.zeros((2*number_sampling*segment, 1))
        samples=[]
        for k in range(0,segment):#index number
            delta_waypoint=(waypoints[k+1]-waypoints[k])/(number_sampling+1)
            print("delta,wp",delta_waypoint)
            for i, t in enumerate(time[k]):
                print("wp at",i," ",t,"",delta_waypoint*(i+1)+waypoints[k])
                mtxh_sample[k*2*number_sampling+2*i] = delta_waypoint*(i+1)+waypoints[k]+corridor
                mtxh_sample[k*2*number_sampling+2*i+1] = -(delta_waypoint*(i+1)+waypoints[k]-corridor)
                #####
                mtxG_sample[k*2*number_sampling+2*i,k*(n_order+1):(k+1)*(n_order+1)]=self.getCoeff(0,n_order,t)
                mtxG_sample[k*2*number_sampling+2*i+1,k*(n_order+1):(k+1)*(n_order+1)]=-np.array(self.getCoeff(0,n_order,t))
                
                samples.append(delta_waypoint*(i+1)+waypoints[k])
                
        mtxG=np.vstack((mtxG,mtxG_sample))
        mtxh=np.vstack((mtxh,mtxh_sample))
        
        

        #4.continuity constraints
        d_order=4
        mtxA_continue = np.zeros((d_order*(segment-1), n_all_poly))
        mtxb_continue = np.zeros((d_order*(segment-1), 1))
        for n in range(1,segment):
            for k in range(0,d_order):
                mtxA_continue[(n-1)*d_order+k, (n-1)*(n_order+1):(n*(n_order+1))] = self.getCoeff(k,n_order,T[n-1])
                mtxA_continue[(n-1)*d_order+k, n*(n_order+1):((n+1)*(n_order+1))]=-np.array(self.getCoeff(k,n_order,0))
        mtxA=np.vstack((mtxA,mtxA_continue))
        mtxb=np.vstack((mtxb,mtxb_continue))
        
        return mtxA,mtxb,mtxG,mtxh,samples

    
    def generateTargetPos(self,control_freq_hz):
        waypoints=np.array(self.waypoints)
        target=[]
        num=0
        segment=len(waypoints)-1
        ################get waypoint of different dimension#####################
        waypointx=np.array(waypoints)[:,0]
        waypointy=np.array(waypoints)[:,1]
        waypointz=np.array(waypoints)[:,2]
        #####################get constraint matrix################################
        Amatx,bmatx,Gmatx,hmatx,samplesx=self.getConstrainMtx(waypointx)
        Amaty,bmaty,Gmaty,hmaty,samplesy=self.getConstrainMtx(waypointy)
        Amatz,bmatz,Gmatz,hmatz,samplesz=self.getConstrainMtx(waypointz)
        #########################get cost function####################
        samples=np.vstack((np.vstack((np.array(samplesx),np.array(samplesy))),np.array(samplesz)))

        self.samples=samples.T.tolist()
        print(self.samples)
        
        Q=self.getmtxQ()
        print(Q.shape)
        Q=cvxopt.matrix(Q)
        ###dimx
        Ax=cvxopt.matrix(Amatx)
        bx=cvxopt.matrix(bmatx)
        q=cvxopt.matrix(np.zeros((np.shape(Amatx)[1],1)))
        Gx=cvxopt.matrix(Gmatx)
        hx=cvxopt.matrix(hmatx)
        solx=cvxopt.solvers.qp(Q,q, Gx,hx,Ax, bx)
        param_x=np.array(solx["x"])
        #print(paramx)
        #rint(paramx.shape)
       # print(np.array(paramx))
        ###dimy
        Ay=cvxopt.matrix(Amaty)
        by=cvxopt.matrix(bmaty)
        Gy=cvxopt.matrix(Gmaty)
        hy=cvxopt.matrix(hmaty)
        q=cvxopt.matrix(np.zeros((np.shape(Amaty)[1],1)))
        soly=cvxopt.solvers.qp(Q,q, Gy,hy,A=Ay, b=by)
        param_y=np.array(soly["x"])
        #print(paramy)
        ###dimz
        Az=cvxopt.matrix(Amatz)
        bz=cvxopt.matrix(bmatz)
        q=cvxopt.matrix(np.zeros((np.shape(Amatz)[1],1)))
        Gz=cvxopt.matrix(Gmatz)
        hz=cvxopt.matrix(hmatz)
        solz=cvxopt.solvers.qp(Q,q, Gz,hz,A=Az, b=bz)
        param_z=np.array(solz["x"])
        #print(paramz)
        
        midpointx=[]
        midpointy=[]
        midpointz=[]
        
        T,S=self.setTime()
        #S.insert(0,0)#head insert
        S=np.insert(S,0,0)
        #print(T,S)
        
        delta_t=1/control_freq_hz
        for i in range(0,segment):
            for t in np.arange(0,T[i]+delta_t,delta_t):
                #time=(t-S[i])/T[i]#rescale to 0-1
                time=t
                #print(time)
                midpointx.append(float(self.getCoeff(0,7,time)@param_x[i*8:(i+1)*8]))
                midpointy.append(float(self.getCoeff(0,7,time)@param_y[i*8:(i+1)*8]))
                midpointz.append(float(self.getCoeff(0,7,time)@param_z[i*8:(i+1)*8]))
                
        target=np.array([[midpointx[i],midpointy[i],midpointz[i]] for i in range(0,len(midpointx))])
        num=len(target)
        
        return target,num
    
    
if __name__ == "__main__":
    ########################################################################
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.loadURDF("plane.urdf")
    waypoints=[[0,2,2],[2,2,2],[2,-2,2],[0,-2,2]]
    smooth=smooth_trajectory(waypoints)
    minimum=minimum_snap(waypoints)
    mini_corridor=minimum_snap_corridor(waypoints)
    #target,num=tj_from_multilines(start_pos,end_pos,control_freq_hz)
    TARGET_POS,NUM_WP=mini_corridor.generateTargetPos(control_freq_hz=40)
    #mini_corridor.sampling()
    p.addUserDebugLine([0,2,2], [2,2,2], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    p.addUserDebugLine([2,2,2], [2,-2,2], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    p.addUserDebugLine([2,-2,2], [0,-2,2], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
   
    for wp in range(0,len(TARGET_POS)-20,20):
        p.addUserDebugLine(TARGET_POS[wp], TARGET_POS[wp+20], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    
    
    corridor=0.7*2
    meshScale = [corridor/5, corridor/5, corridor/5]
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="cube.obj",
                                    rgbaColor=[0, 0, 1, 0.5],
                                    specularColor=[0.4, .4, 0],
                                    meshScale=meshScale)

    ###########################add around waypoints#################################
    segment=len(waypoints)-1
    
    for k in range(0,segment-1):#index number
        p.createMultiBody(baseMass=0,
                            baseInertialFramePosition=[0, 0, 0],
                            baseVisualShapeIndex=visualShapeId,
                            basePosition=waypoints[k+1],
                            useMaximalCoordinates=True)
        
    corridor=0.7*2
    meshScale = [corridor, corridor, corridor]
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="cube.obj",
                                    rgbaColor=[1, 1, 1, 0.5],
                                    specularColor=[0.4, .4, 0],
                                    meshScale=meshScale)
    
    samples=list(mini_corridor.samples)
    
    print(samples)
    for k in range(0,len(samples)):#index number
        p.createMultiBody(baseMass=0,
                            baseInertialFramePosition=[0, 0, 0],
                            baseVisualShapeIndex=visualShapeId,
                            basePosition=samples[k],
                            useMaximalCoordinates=True)
    
    while 1:
        p.stepSimulation()