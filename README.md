# PDM-project
 Quadrotor project RO47005 Planning and Decision making 2022-23
 Group10

## Installation
The environment gym-pybullet-drones is used. Please follow their installation guide first:
https://github.com/utiasDSL/gym-pybullet-drones

CVXOPT is also used.Please follow their installation guide first:
https://cvxopt.org/install/index.html

Then clone this repository.

```sh
$ git clone git@github.com:nanaminh/PDM-project.git
```
To run all the sample code, please remember to first activate the drones environment.
```sh
$ conda activate drones

```

## Our work
We implement RRT, RRT* and informed RRT* from scratch.
To run them separatelyin a blanc environment without flying drones: 
```sh
$ python3 base_rrt.py 

```
```sh
$ python3 rrt_star.py 

```
```sh
$ python3 informed_rrt_star.py 

```
To run informed rrt star with updating path and goal points.
```sh
$ python3 super_cool.py 

```




We implement minimum snap, corridor minimum snap and closed-form minimum_snap trajectory optimization from scratch.
To compare  different trajectories in a blanc environment without flying drones: 
```sh
$ python3 minimum_snap_corridor.py 

```
To combine  different trajectories with waypoints found by RRT method and  fly a drone: 
```sh
$ python3 compare_trajectory.py
```

To run Bang-(Coast)-Bang Segments
```sh
$ python3 trajectory_generation_test.py 
```

![Image text](https://github.com/nanaminh/PDM-project/blob/main/IMG/corridors.jpg)

![Image text](https://github.com/nanaminh/PDM-project/blob/main/IMG/basic_rrt.png)

![Image text](https://github.com/nanaminh/PDM-project/blob/main/IMG/trajectory_generation_test.png)




