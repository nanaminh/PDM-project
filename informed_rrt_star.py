#######
# RO47005 Planning and decision-making 22/23
# Group:10
# Author: Abel van Elburg
# email: a.t.g.vanelburg@student.tudelft.nl
#######
import pybullet as p
import pybullet_data as pd
from informed_sampling import *


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


class InformedRRTStar:
    """
    RRT*

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
        self.shortest_path = -1  # distance from goal to root, -1 if no path is found

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
        # 1. Get random sample if goal is not found, otherwise use informed sampling.
        
        if not self.goal_found:
            # a random point is sampled in an ellipsoid with preset margin around the start and goal 
            margin = 2.0
            random_position = informed_sample(np.array(self.start), np.array(self.goal), np.linalg.norm(np.array(self.start) - np.array(self.goal)) + margin)
        else:
            random_position = informed_sample(np.array(self.start), np.array(self.goal), self.tree[self.goal_index].dist)
        # 2. Find the nearest node in the tree
        min_distance = 100000
        min_index = 0
        min_position = self.start
        for i in range(0, self.index + 1):
            diff_coordinate = random_position - self.tree[i].position
            euler_distance = np.linalg.norm(diff_coordinate)
            if euler_distance < min_distance:
                min_distance = euler_distance
                min_index = i
                min_position = diff_coordinate
        node_nearest = self.tree[min_index]

        # 3. Check if the path to the node is collision free
        collision = collision_check(node_nearest.position, random_position)
        if not collision:
            # 4. Push new node to the tree
            # normalized = diff_coordinate / euler_distance * self.delta
            new_position = node_nearest.position + min_position
            node_new = Node(new_position, min_index, node_nearest.dist + min_distance)  # generate node_new
            self.push_new_node(node_new)

            # 5. Search other possible parents
            nodes_near_list = [min_index]
            near_count = 0
            near_index = min_index
            shortest_dist = node_new.dist
            neighbourhood_radius = 1
            for i in range(0, self.index + 1):
                node_near_temp = self.tree[i]
                node_near_dist = np.linalg.norm(node_near_temp.position - node_new.position)
                if node_near_dist < neighbourhood_radius:
                    collision = collision_check(node_near_temp.position, node_new.position)
                    if not collision:
                        near_count += 1
                        nodes_near_list.append(i)
                        temp_dist = node_near_dist + node_near_temp.dist
                        if temp_dist < shortest_dist:
                            shortest_dist = temp_dist
                            near_index = i

            # 6. (Re-)attach the node to the best parent
            node_near = self.tree[near_index]
            node_new.index_parent = near_index
            node_new.dist = shortest_dist
            visualisation(list(node_near.position), list(new_position))

            # 7. Rewiring nodes to new node (if distance becomes shorter)
            for i in range(near_count):
                node = self.tree[nodes_near_list[i]]
                current_dist = node.dist
                new_dist = node_new.dist + np.linalg.norm(node.position - node_new.position)
                if new_dist < current_dist:
                    node.index_parent = self.index
                    node.dist = new_dist
                    p.addUserDebugLine(node.position, node_new.position, lineColorRGB=[0, 0, 1], lifeTime=0,
                                       lineWidth=0.5)

            # 8. Check whether the goal is reached, and if it is an improvement
            distance_to_goal = np.linalg.norm(new_position - np.array(self.goal))
            if distance_to_goal < self.threshold:
                if self.first_arrive:  # push the goal into the tree when arrive at first time, and backtrace
                    print("Goal FOUND!")
                    self.goal_found = True

                    goal_node = Node(np.array(self.goal), self.index,
                                     node_new.dist + distance_to_goal)  # the goal's parent is the new node
                    self.push_new_node(goal_node)
                    self.goal_index = self.index
                    self.shortest_path = goal_node.dist
                    visualisation(list(new_position), self.goal)

                    # 9a. Trace back to start
                    self.backtracing()
                    self.first_arrive = False
                else:
                    new_goal_node = Node(np.array(self.goal), self.index,
                                         node_new.dist + distance_to_goal)
                    if new_goal_node.dist < self.shortest_path:
                        print("better path found!")
                        self.push_new_node(new_goal_node)
                        self.goal_index = self.index
                        self.shortest_path = new_goal_node.dist
                        visualisation(list(new_position), self.goal)

                        # 9b. Trace back to start
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
                               lineColorRGB=[0, 1, 0], lifeTime=0, lineWidth=3.0)
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

    # debug test


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    # p.configureDebugVisualizer(p. COV_ENABLE_WIREFRAME, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    
    
    # p.loadURDF("plane.urdf")
    
    quarterRotation = [0, 0, 0.7071, 0.7071]
    
    # WINDOWS
    windowsCenter = np.array([3,0,0])
    p.loadURDF("windowsHorizontal.urdf", np.add(windowsCenter,np.array([0, 0, -0.1])))
    p.loadURDF("windowsHorizontal.urdf", np.add(windowsCenter,np.array([0, 0, 1.6])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter,np.array([0, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter,np.array([-1, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter,np.array([1, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter,np.array([-2, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter,np.array([2, 0, 0.1])))
    # TUNNEL
    tunnelCenter = np.array([0,-2,0.1])
    p.loadURDF("lowWall.urdf", np.add(tunnelCenter,np.array([0, -0.4, -0.1])))
    p.loadURDF("roof.urdf", np.add(tunnelCenter,np.array([0, 0, -0.1])))
    p.loadURDF("lowWall.urdf", np.add(tunnelCenter,np.array([0, 0.4, -0.1])))
    # LOW WALL
    p.loadURDF("lowWall.urdf", [-2,0,0.1])
    # CRAWL
    crawlCenter = np.array([0,3,-1])
    p.loadURDF("windowsHorizontal.urdf", np.add(crawlCenter,np.array([0, 0, 1.5])), quarterRotation)
    p.loadURDF("windowsVertical.urdf", np.add(crawlCenter,np.array([0, -2, 0])), quarterRotation)
    p.loadURDF("windowsVertical.urdf", np.add(crawlCenter,np.array([0, 2, 0])), quarterRotation)

    # Floor collider
    p.loadURDF("floorCollider.urdf", [0,0,-0.5])
    
    
    windowsSeq = np.array([[3, 3, 0.5], [-3, -3, 0.5]])
    tunnelSeq = np.array([[3, -2, 0.5], [-3, -2, 0.5]])
    lowWallSeq = np.array([[-2, -2, 0.5], [-3, 2, 0.5]])
    crawlSeq = np.array([[-2, 2, 0.5], [2, 2, 0.5]])
    
    info_rrt_star = InformedRRTStar(lowWallSeq[0], lowWallSeq[1])   

	
    ######################DEBUG TEST1 stop after goal found###############################
    while 1:
        while info_rrt_star.index <= 1000:  # Limit in the amount of nodes.
            info_rrt_star.step()
    ######################DEBUG TEST1 END#######################################################

    print("FINISHED")
    print("Shortest path found: ", info_rrt_star.tree[info_rrt_star.goal_index].dist)
