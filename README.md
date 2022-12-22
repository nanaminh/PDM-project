# PDM-project
 Quadrotor project RO47005 Planning and Decision making 2022-23
 Group10

## Install
Please clone this repository and under the repository of gym-pybullet `~/gym-pybullet-drones/gym_pybullet_drones/`.

```sh
$ cd gym-pybullet-drones/gym_pybullet_drones/
$ git clone git@github.com:nanaminh/PDM-project.git
```
To run the basic_rrt in a blanc environment:
```sh
$ python3 base_rrt.py 
```
To run the basic_rrt in an environment with obstacles:
```sh
$ python3 base_rrt_test.py 
```
WARNING: 
In order to make the teast easier, the gym step function`base_rrt_test.py`is commented, and there will be BUG after the simulation terminated, because of the usage of logger.save(). It will be debuged after generating feasible trajectory.
