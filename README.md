# PDM-project
 Quadrotor project RO47005 Planning and Decision making 2022-23
 Group10

## Install
22/12/2022
Please clone this repository and under the repository of gym-pybullet `~/gym-pybullet-drones/gym_pybullet_drones/`.

```sh
$ cd gym-pybullet-drones/gym_pybullet_drones/
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
WARNING: 

1.In order to make the test easier, the gym step function in `base_rrt_test.py`is commented, and there will be BUG after the simulation terminated, because of the usage of logger.save(). It will be debuged after generating feasible trajectory.

2.The trajectory generation is naive method. The drone cannot follow it perfectly.


