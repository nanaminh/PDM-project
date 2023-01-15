#######
# RO47005 Planning and decision-making 22/23
# Group:10
# Aurthor: Danning Zhao
# email: D.ZHAO-3@student.tudelft.nl
#######
import numpy as np
import pybullet as p
import pybullet_data as pd


class Node:
    """
    a structure contains information of RRT tree nodes
    coordinates x,y,z
    index of the node's parent
    """

    def __init__(self, position, parent, dist=0):
        self.position = position  # coordinate (x,y,z),np.array
        self.index_parent = parent
        self.dist = dist  # dist to root


def visualisation(start_pos, goal_pos):
    p.addUserDebugLine(start_pos, goal_pos, lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=0.5)


def collision_check(start_pos, goal_pos):
    collision = False
    result = p.rayTest(list(start_pos), list(goal_pos))[0][
        0]  # get the collision object id, if==-1, then no collision
    # print(result)
    if result != -1:
        collision = True
    return collision


class RRT:
    """
    basic RRT 

    """

    def __init__(self, start_pos, goal_pos):
        """
        initialize RRT tree with a root rrt_node
        input: start_pos[x,y,z], goal_pos[x,y,z]
        
        the nodes of tree is stored in an array
        in order to simplify the random sample process, set coordinates of goal_pos>start_pos>0,
        
        TO DO: improve the data structure with Kd-tree
        TO DO: improve the random sample process to accept abitrary start_pos and goal_pos
        TO DO: tune the self.delta
        TO DO: visualize the start position and goal position with point
        TO DO: inverse the traceback list
        """
        self.start = start_pos
        self.goal = goal_pos
        self.index = -1  # the index of current node, -1 means no nodes in the list
        self.goal_found = False
        self.delta = 0.3  # need fine-tune
        self.threshold = 0.5  # threshold to the goal

        self.first_arrive = True
        self.goal_index = -1  # the index of goal node, for back tracing
        self.tree = []

        ####################################
        self.trajectory_back = []  #
        self.waypoint = []
        self.start_pos = []
        self.end_pos = []
        #########################################

        root = Node(position=np.array(start_pos), parent=-1)  # the root node of RRT tree
        self.push_new_node(root)  # push the root, index=0
        # visualize the start and goal position with a line
        print("start and goal", self.start, self.goal)
        p.addUserDebugLine(self.start, self.goal, lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)

    def step(self):
        # 1.random sample
        random_position = np.array([np.random.random_sample() * (self.goal[i]+1 - self.start[i])  for i in
                                    range(0, 3)])  # +1 is hard coded, for a larger sample space
        print(random_position)
        # 2.find the nearest node in the tree
        min_distance = 100000
        min_index = 0
        for i in range(0, self.index + 1):
            diff_coordinate = random_position - self.tree[i].position
            euler_distance = np.linalg.norm(diff_coordinate)
            if euler_distance < min_distance:
                min_distance = euler_distance
                min_index = i

        node_nearest = self.tree[min_index]

        # 3.check collision free
        collision = collision_check(node_nearest.position, random_position)
        if not collision:
            # 4.push the new node into the tree
            normalized = diff_coordinate / euler_distance * self.delta
            new_position = node_nearest.position + normalized
            node_new = Node(new_position, min_index, node_nearest.dist + self.delta)  # generate node_new
            self.push_new_node(node_new)
            visualisation(list(node_nearest.position), list(new_position))

            # print("index",self.index)
            # 5.check whether the goal is reached
            distance_to_goal = np.linalg.norm(new_position - np.array(self.goal))
            if distance_to_goal < self.threshold:
                print("Goal FOUND!")
                if self.first_arrive:  # push the goal into the tree when arrive at first time, and backtrace
                    goal_node = Node(np.array(self.goal), self.index,
                                     node_new.dist + distance_to_goal)  # the goal's parent is the new node
                    self.push_new_node(goal_node)
                    visualisation(list(new_position), self.goal)

                    self.goal_found = True
                    self.first_arrive = False
                    self.goal_index = self.index

                    # 6.trace back
                    self.backtracing()

        # return self.goal_found

    def push_new_node(self, node):
        self.tree.append(node)
        self.index += 1  # update the current index

    def backtracing(self):
        """
        draw the trajectory from goal to root and generate the list of node

        """
        index = self.goal_index
        self.trajectory_back = []
        self.waypoint = []
        self.start_pos = []
        self.end_pos = []

        while index != 0:  # if arrive root, break
            self.trajectory_back.append(self.tree[index])
            parent_index = self.tree[index].index_parent
            p.addUserDebugLine(list(self.tree[parent_index].position), list(self.tree[index].position),
                               lineColorRGB=[0, 1, 0], lifeTime=0, lineWidth=4)
            index = parent_index

        self.trajectory_back.append(self.tree[0])  # push the root node
        self.trajectory_back.reverse()
        # update waypoint information
        self.get_waypoints()

        print("straight distance between start and goal: ", np.linalg.norm(np.array(self.start) - np.array(self.goal)))
        print("trajectory total length: ", self.tree[self.goal_index].dist)

    def get_waypoints(self):
        """
        output:waypoint,start_pos,end_pos  np.array()
        
        """
        if self.trajectory_back == []:
            raise ValueError("Path Still NOT FOUND!")

        waypoint = np.vstack([node.position for node in self.trajectory_back])
        self.waypoint = waypoint
        self.start_pos = waypoint[0:-1]
        self.end_pos = waypoint[1:]
        print(np.shape(self.waypoint))
        print(np.shape(self.start_pos))
        print(np.shape(self.end_pos))


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

    rrt = RRT([0.5, 0.5, 0.5], [4, 4, 4])

    ######################DEBUG TEST1 stop after goal found###############################
    while 1:
        if not rrt.goal_found:  # if the goal haven't been found
            rrt.step()
            
        # print(np.array(rrt.trajectory_back))

    ######################DEBUG TEST1 END#######################################################
