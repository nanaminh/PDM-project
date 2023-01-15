#######
# RO47005 Planning and decision-making 22/23
# Group:10
#######
import pybullet as p
import pybullet_data as pd
from informed_sampling import *
import time

start_time = time.time()


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


def print_results(pathfinder):
    if pathfinder.goal_found:
        path_length = pathfinder.tree[pathfinder.goal_index].dist
        # print new path length and elapsed time
        if pathfinder.last_path_length != path_length:
            elapsed_time = time.time() - start_time
            # print("after", elapsed_time, "seconds the shortest path =", pathLength)
            print(elapsed_time, ",", path_length, "")
            pathfinder.last_path_length = path_length


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

    def __init__(self, start_pos, goal_pos, first_path=True, previous=None):
        """
        initialize RRT tree with a root rrt_node
        input: start_pos[x,y,z], goal_pos[x,y,z]
        
        the nodes of tree are stored in an array
        in order to simplify the random sample process, set coordinates of goal_pos>start_pos>0,
        """
        self.last_path_length = None
        self.shortest_path = None
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
        self.previous = previous
        self.first_path = first_path
        if self.first_path:
            self.trajectory_back = []  #
            self.waypoint = []
            self.start_pos = []
            self.end_pos = []
        else:
            previous = self.previous
            self.trajectory_back = previous.trajectory_back  #
            self.waypoint = previous.waypoint
            self.start_pos = previous.start_pos
            self.end_pos = previous.end_pos
        #########################################

        root = Node(position=np.array(start_pos), parent=-1)  # the root node of RRT tree
        self.push_new_node(root)  # push the root, index=0
        # visualize the start and goal position with a line
        initlTime = time.time() - start_time
        print(initlTime,", 0")
        # print("start and goal", self.start, self.goal)
        p.addUserDebugLine(self.start, self.goal, lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)

    def step(self):
        # 1.random sample
        # a random point is sampled in an ellipsoid with preset margin around the start and goal 
        margin = 2.0
        random_position = informed_sample(np.array(self.start), np.array(self.goal),
                                          np.linalg.norm(np.array(self.start) - np.array(self.goal)) + margin)
        # print(random_position)
        # 2.find the nearest node in the tree
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

        # 3.check collision free
        collision = collision_check(node_nearest.position, random_position)
        #print(collision)
        if not collision:
            # 4.push the new node into the tree
            new_position = node_nearest.position + min_position
            node_new = Node(new_position, min_index, node_nearest.dist + min_distance)  # generate node_new
            self.push_new_node(node_new)
            # visualisation(list(node_nearest.position), list(new_position))

            # print("index",self.index)
            # 5.check whether the goal is reached
            distance_to_goal = np.linalg.norm(new_position - np.array(self.goal))
            if distance_to_goal < self.threshold:
                if self.first_arrive:  # push the goal into the tree when arrive at first time, and backtrace
                    # print("Goal FOUND!")
                    self.goal_found = True

                    goal_node = Node(np.array(self.goal), self.index,
                                     node_new.dist + distance_to_goal)  # the goal's parent is the new node
                    self.push_new_node(goal_node)
                    self.goal_index = self.index
                    self.shortest_path = goal_node.dist
                    # visualisation(list(new_position), self.goal)

                    # 9a. Trace back to start
                    self.backtracking()
                    self.first_arrive = False
                else:
                    new_goal_node = Node(np.array(self.goal), self.index,
                                         node_new.dist + distance_to_goal)
                    if new_goal_node.dist < self.shortest_path:
                        # print("better path found!")
                        self.push_new_node(new_goal_node)
                        self.goal_index = self.index
                        self.shortest_path = new_goal_node.dist
                        # visualisation(list(new_position), self.goal)

                        # 9b. Trace back to start
                        self.backtracking()

        # return self.goal_found

    def push_new_node(self, node):
        self.tree.append(node)
        self.index += 1  # update the current index

    def backtracking(self):
        """
        draw the trajectory from goal to root and generate the list of node

        """
        index = self.goal_index
        if self.first_path:
            self.trajectory_back = []  #
            self.waypoint = []
            self.start_pos = []
            self.end_pos = []
        else:
            previous = self.previous
            self.trajectory_back = previous.trajectory_back  #
            self.waypoint = previous.waypoint
            self.start_pos = previous.start_pos
            self.end_pos = previous.end_pos

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

        # print("straight distance between start and goal: ", np.linalg.norm(np.array(self.start) - np.array(self.goal)))
        # print("trajectory total length: ", self.tree[self.goal_index].dist)

    def get_waypoints(self):
        """
        output:waypoint,start_pos,end_pos  np.array()
        
        """
        if not self.trajectory_back:
            raise ValueError("Path Still NOT FOUND!")

        waypoint = np.vstack([node.position for node in self.trajectory_back])
        self.waypoint = waypoint
        self.start_pos = waypoint[0:-1]
        self.end_pos = waypoint[1:]
        # print(np.shape(self.waypoint))
        # print(np.shape(self.start_pos))
        # print(np.shape(self.end_pos))


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())

    quarterRotation = [0, 0, 0.7071, 0.7071]

    # WINDOWS
    windowsCenter = np.array([3, 0, 0])
    p.loadURDF("windowsHorizontal.urdf", np.add(windowsCenter, np.array([0, 0, -0.1])))
    p.loadURDF("windowsHorizontal.urdf", np.add(windowsCenter, np.array([0, 0, 1.6])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter, np.array([0, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter, np.array([-1, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter, np.array([1, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter, np.array([-2, 0, 0.1])))
    p.loadURDF("windowsVertical.urdf", np.add(windowsCenter, np.array([2, 0, 0.1])))
    # TUNNEL
    tunnelCenter = np.array([0, -2, 0.1])
    p.loadURDF("lowWall.urdf", np.add(tunnelCenter, np.array([0, -0.4, -0.1])))
    p.loadURDF("roof.urdf", np.add(tunnelCenter, np.array([0, 0, -0.1])))
    p.loadURDF("lowWall.urdf", np.add(tunnelCenter, np.array([0, 0.4, -0.1])))
    # LOW WALL
    p.loadURDF("lowWall.urdf", [-2, 0, 0.1])
    # CRAWL
    crawlCenter = np.array([0, 3, -1])
    p.loadURDF("windowsHorizontal.urdf", np.add(crawlCenter, np.array([0, 0, 1.5])), quarterRotation)
    p.loadURDF("windowsVertical.urdf", np.add(crawlCenter, np.array([0, -2, 0])), quarterRotation)
    p.loadURDF("windowsVertical.urdf", np.add(crawlCenter, np.array([0, 2, 0])), quarterRotation)

    # Floor collider
    p.loadURDF("floorCollider.urdf", [0, 0, -0.5])

    lowWallSeq = np.array([[-2, -2, 0.5], [-2, 2, 0.5]])
    crawlSeq = np.array([[-2, 2, 0.5], [3, 2, 0.5]])
    windowsSeq = np.array([[3, 2, 0.5], [3, -2, 0.5]])
    tunnelSeq = np.array([[3, -2, 0.5], [-2, -2, 0.5]])

    shortest_distance = 0
    shortest_path = 0

    rrt = RRT(lowWallSeq[0], lowWallSeq[1])
    while rrt.index <= 1000:  # Limit in the amount of nodes.
        rrt.step()
        print_results(rrt)
    shortest_path += rrt.shortest_path
    shortest_distance += np.linalg.norm(np.array(rrt.start) - np.array(rrt.goal))
    prev = rrt

    rrt = RRT(crawlSeq[0], crawlSeq[1], first_path=False, previous=prev)
    while rrt.index <= 1000:
        rrt.step()
        print_results(rrt)
    shortest_path += rrt.shortest_path
    shortest_distance += np.linalg.norm(np.array(rrt.start) - np.array(rrt.goal))
    prev = rrt

    rrt = RRT(windowsSeq[0], windowsSeq[1], first_path=False, previous=prev)
    while rrt.index <= 1000:
        rrt.step()
        print_results(rrt)
    shortest_path += rrt.shortest_path
    shortest_distance += np.linalg.norm(np.array(rrt.start) - np.array(rrt.goal))
    prev = rrt

    rrt = RRT(tunnelSeq[0], tunnelSeq[1], first_path=False, previous=prev)
    while rrt.index <= 1000:
        rrt.step()
        print_results(rrt)
    shortest_path += rrt.shortest_path
    shortest_distance += np.linalg.norm(np.array(rrt.start) - np.array(rrt.goal))

    print(time.time()-start_time,rrt.tree[rrt.goal_index].dist)
    print("FINISHED")
    # print("Shortest distance possible: ", shortest_distance)
    # print("Shortest path found: ", shortest_path)
