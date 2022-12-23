import numpy as np
import pybullet as p
import pybullet_data as pd
#######
#RO47005 Planning and decision making 22/23
#Group:10
#Aurthor: Danning Zhao
#email: D.ZHAO-3@student.tudelft.nl
#Reference to RO47001 Robot Dynamics & Control assigment3 template code
#######

    # PERIOD = 10###
    # NUM_WP = control_freq_hz*PERIOD# get all the weaypoints
    # TARGET_POS = np.zeros((NUM_WP,3))

    # for i in range(NUM_WP):
    #     TARGET_POS[i, :] = -i*0.005,-i*0.005,i*0.005

def tj_from_line(start_pos,end_pos,t_ttl,control_freq_hz):
    """
    input: start_pos,end_pos  list[x,y,z]coordinate
    input: t_ttl  total time of this trajectory scalar
    output: desired_pos   array
    """
    PERIOD=t_ttl
    NUM_WP = int(np.ceil(control_freq_hz*PERIOD))
    print("sub traj",NUM_WP)
    TARGET_POS = np.zeros((NUM_WP,3))
    start_pos=np.array(start_pos)
    end_pos=np.array(end_pos)
    
    v_max=(end_pos-start_pos)*2/PERIOD 
    
    
    for i in range(0,NUM_WP):
        t_c=i*1/control_freq_hz#current time
        if t_c>=0  and t_c<PERIOD/2:
            vel = v_max*t_c/(PERIOD/2)
            pos = start_pos + t_c*vel/2
            TARGET_POS[i,:]=pos#acc = [0,0,0]
        else:
            vel = v_max*(PERIOD-t_c)/( PERIOD/2)
            pos = end_pos - (PERIOD-t_c)*vel/2
            TARGET_POS[i,:]=pos
        #acc = [0,0,0]
    # desired_position["vel"]=vel
    # desired_position["pos"]=pos
    # desired_position["acc"]=acc
    return TARGET_POS

def tj_from_multilines(start_pos,end_pos,control_freq_hz):
    """
    input: start_pos,end_pos  waypoints,shape[num_waypoints,3]
    output: desired_pos   array
    """
    if np.shape(start_pos)!=np.shape(end_pos):
        raise ValueError("start_pos should be SMAE SHAPE with end_pos")
    
    num_segment=np.shape(start_pos)[0]
    start_pos=np.array(start_pos)
    end_pos=np.array(end_pos)
    t_ttl=np.sqrt(np.sum((end_pos-start_pos)**2,axis=1))#naive way to assign time
    TARGET_POS=[]
    TARGET_POS=np.vstack([tj_from_line(start_pos[i],end_pos[i],t_ttl[i],control_freq_hz) for i in range(num_segment)])
    # for i in range(num_segment):
    #     TARGET_POS.append(tj_from_line(start_pos[i],end_pos[i],t_ttl[i],control_freq_hz))
    #     print(i)
    NUM_WP = int(sum(np.ceil(control_freq_hz*t_ttl)))
    print("TARGET_POS shape",np.shape(TARGET_POS))
    return TARGET_POS, NUM_WP
   
    #deubg test
if __name__ == "__main__":
    # rrt=basic_rrt([0,0,0],[1,1,1])
    # print(rrt.tree)
    # print(rrt.index)
    # #print(rrt_node.__doc__)
    # start=[0,0,0]
    # goal=[4,.4,33]
    # print(goal-start)
    
    # p.connect(p.GUI)
    # p.setAdditionalSearchPath(pd.getDataPath())
    # # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
    # # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # p.loadURDF("plane.urdf")
    # p.addUserDebugLine([0,0,0], [1,1,1], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    # p.addUserDebugLine( [1,1,1],[3,1,2], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    # p.addUserDebugLine([3,1,2], [5,4,1], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    # p.addUserDebugLine([5,4,1], [6,7,2], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)

    
    # ######################DEBUG TEST1 stop after goal found###############################
    # while 1:

    #     p.stepSimulation()
    #######NUMPY TEST################################
    a=np.array([1,2,3])
    print(a*a)
    ##################FUNCTION TEST############################
    target_pos=tj_from_line([0,0,0],[1,1,1],4,10)
    #print(target_pos)
    
    b=[[1,1,1],[1,1,1]]
    #print(np.shape(b))
    ###################Multipline function test#################
    start_pos=[[0,0,0],[1,1,1],[0,2,3]]
    end_pos=[[1,1,1],[0,2,3],[4,5,6]]
    #print(start_pos[0])
    target,nm=tj_from_multilines(start_pos,end_pos,24)
    print( nm)