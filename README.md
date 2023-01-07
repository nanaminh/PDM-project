# PDM-project
 Quadrotor project RO47005 Planning and Decision making 2022-23
 Group10

## Install
22/12/2022
For this project the existing environment gym-pybullet-drones was used. Please follow their installation guide first:
https://github.com/utiasDSL/gym-pybullet-drones

After this is done and confirmed by running their examples, clone this repository in the same folder as the environment.

```sh
$ git cd '$example$/GitHub/'
$ git clone git@github.com:nanaminh/PDM-project.git
```
To run the basic_rrt in a blanc environment:
```sh
$ python3 base_rrt.py 

```
23/12/2022
To run simple trajectory generation in a blanc environment:
```sh
$ python3 bang_bang.py 

```
To test simple trajectory and simple target position generation:
```sh
$ python3 trajectory_generation_test.py 

```
To run the basic_rrt in an environment with obstacles and the drone will follow the trajectory:
```sh
$ python3 base_rrt_test.py 
```
![Image text](https://github.com/nanaminh/PDM-project/blob/main/IMG/basic_rrt.png)

![Image text](https://github.com/nanaminh/PDM-project/blob/main/IMG/basic_rrt_follow2.png)

![Image text](https://github.com/nanaminh/PDM-project/blob/main/IMG/trajectory_generation_test.png)

07/01/2023
To run the smooth trajectory in an environment without obstacles:
```sh
$ python3 smooth_trajectory.py 
```
To run the smooth_trajectory in an environment with obstacles and the drone will follow the trajectory:
```sh
$ python3 smooth_trajectory_test.py 
```

~~WARNING: 

1.In order to make the test easier, the gym step function in `base_rrt_test.py`is commented, and there will be BUG after the simulation terminated, because of the usage of logger.save(). It will be debuged after generating feasible trajectory.

2.The trajectory generation is naive method. The drone cannot follow it perfectly.~~


