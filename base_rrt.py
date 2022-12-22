#######
#RO47005 Planning and decision making 22/23
#Group:10
#Aurthor: Danning Zhao
#email: D.ZHAO-3@student.tudelft.nl
#######
import numpy as np
import pybullet as p
import pybullet_data as pd
import time

class rrt_node():
    """
    a structure contains information of RRT tree nodes
    coordiantes x,y,z
    index of the node's parent
    """
    def __init__(self,position,parent):
        self.position=position#coordinate (x,y,z),np.array
        self.index_parent=parent

class basic_rrt():
    """
    basic RRT 

    """
    
    def __init__(self,start_pos, goal_pos):
        """
        initialize RRT tree with a root rrt_node
        input: start_pos[x,y,z], goal_pos[x,y,z]
        
        the nodes of tree is stored in an array
        in order to simplify the random sample process, set coordinates of goal_pos>start_pos>0,
        
        TO DO: improve the data structure with Kd-tree
        TO DO: improve the random sample process to accept abitrary start_pos and goal_pos
        TO DO: tune the self.delta
        TO DO: visualize the start position and goal position with point
        """
        self.start=start_pos
        self.goal=goal_pos
        self.index=-1# the index of current node, -1 means no nodes in the list
        self.goal_found=False
        self.delta=0.3#need fine-tune
        self.threshold=0.2#threshol to the goal
        self.tree=[]
        root=rrt_node(position=np.array(start_pos),parent=-1)#the root node of RRT tree
        self.push_newnode(root)#push the root, index=1
        ##visualize the start and goal position with a line
        print("start and goal",self.start,self.goal)
        p.addUserDebugLine(self.start, self.goal, lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
        
        
        
    def step(self):
        #1.random sample
        random_position=np.array([np.random.random_sample()*(self.goal[i]-self.start[i])+1 for i in range(0,3)])#+1 is for a larger sample space
        #print("random_position",random_position)
        #2.find nearest node in the tree
        min_distance=100000
        min_index=0
        for i in range(0,self.index+1):
            diff_coordinate=random_position-self.tree[i].position
            euler_distance=np.linalg.norm(diff_coordinate)
            if euler_distance<min_distance:
                min_distance=euler_distance
                min_index=i
                
        node_nearest=self.tree[min_index]
    
        #3.check collision free
        collision=self.collision_check(node_nearest.position,random_position)
        if collision==False:
            #4.push the new node into the tree
            normalized=diff_coordinate/euler_distance*self.delta
            new_position=node_nearest.position+normalized
            node_new=rrt_node(new_position,min_index)#generate node_new
            self.push_newnode(node_new)
            self.visualisation(list(node_nearest.position),list(new_position))
            
        #print("index",self.index)
        
        
        return self.goal_found
    
    def push_newnode(self,node):
        self.tree.append(node)
        self.index+=1#update the current index
        
    def collision_check(self,start_pos,goal_pos):
        collision=False
        result=p.rayTest(list(start_pos),list(goal_pos))[0][0]#get the collision object id, if==-1, then no collision
        print(result)
        if result!=-1:
            collision=True
        return collision
    
    def visualisation(self,start_pos,goal_pos):
        p.addUserDebugLine(start_pos, goal_pos, lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
        return 0
    
    def backtracing(self):
        return 0
    
    
    #deubg test
if __name__ == "__main__":
    # rrt=basic_rrt([0,0,0],[1,1,1])
    # print(rrt.tree)
    # print(rrt.index)
    # #print(rrt_node.__doc__)
    # start=[0,0,0]
    # goal=[4,.4,33]
    # print(goal-start)
    
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.loadURDF("plane.urdf")

    rrt=basic_rrt([0.5,0.5,0.5],[4,4,4])
    while 1:

        #time.sleep(0.05)
        rrt.step()
        p.stepSimulation()
        
