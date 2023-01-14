#######
#RO47005 Planning and decision making 22/23
#Group:10
#Aurthor: Danning Zhao
#email: D.ZHAO-3@student.tudelft.nl
#Reference to fly.py

#######

import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

###############################################
from bang_bang import tj_from_multilines
from smooth_trajectory_original import smooth_trajectory
from minimum_snap import minimum_snap
###############################################

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_VISION = False
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_AGGREGATE = True
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 240#?
DEFAULT_CONTROL_FREQ_HZ = 48#?
DEFAULT_DURATION_SEC = 100
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        vision=DEFAULT_VISION,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        aggregate=DEFAULT_AGGREGATE,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    #### Initialize the simulation #############################
    H = .1
    H_STEP = .05
    R = .3
    INIT_XYZS = np.array([[0, 0, 0.5] for i in range(num_drones)])#WARNINFG: nested lists, even for num_drones=1
    print("INIT_XYZS shape",np.shape(INIT_XYZS))#WARNING SHAPE INIT_XYZS [[0 0 0]]
    print("INIT_XYZS", INIT_XYZS)
    INIT_RPYS = np.array([[0, 0, 0] for i in range(num_drones)])
    print("INIT_RPYS shape",np.shape(INIT_RPYS))#
    AGGR_PHY_STEPS = int(simulation_freq_hz/control_freq_hz) if aggregate else 1#
    print("AGGR_PHY_STEPS",AGGR_PHY_STEPS)
    
    # INIT_XYZS shape (1, 3)
    # INIT_RPYS shape (1, 3)
    # AGGR_PHY_STEPS 5
    # TARGET_POS SHAPE (480, 3)
    
    #AGGR_PHY_STEPS 5
    #### Initialize a circular trajectory ######################
    # PERIOD = 10###
    # NUM_WP = control_freq_hz*PERIOD# get all the weaypoints
    # TARGET_POS = np.zeros((NUM_WP,3))

    # start_pos=[[0,0,0],[-1,-1,1],[1,2,1.5],[3,1,2]]
    # end_pos=[[-1,-1,1],[1,2,1.5],[3,1,2],[1,-1,2.5]]
    # #print(start_pos[0])
    
    
    waypoints=[[0,0,0],[-1,-1,1],[1,2,1.5],[3,1,2],[1,-1,2.5]]
    smooth=smooth_trajectory(waypoints)
    minimum=minimum_snap(waypoints)
    #target,num=tj_from_multilines(start_pos,end_pos,control_freq_hz)
    TARGET_POS,NUM_WP=smooth.generateTargetPos(control_freq_hz)
    TARGET_POS2,NUM_WP2=minimum.generateTargetPos(control_freq_hz)
    #####################################################################################
    print("TARGET_POS SHAPE", np.shape(TARGET_POS))#TARGET_POS SHAPE (480, 3)  (NUM_WP,3)
    print("target",TARGET_POS)
    wp_counters = np.array([int((i*NUM_WP/6)%NUM_WP) for i in range(num_drones)])#
    print("wp_counters",wp_counters)#[  0  80 160]
    


    #### Create the environment with or without video capture ##
    if vision: 
        env = VisionAviary(drone_model=drone,
                           num_drones=num_drones,
                           initial_xyzs=INIT_XYZS,
                           initial_rpys=INIT_RPYS,
                           physics=physics,
                           neighbourhood_radius=10,
                           freq=simulation_freq_hz,
                           aggregate_phy_steps=AGGR_PHY_STEPS,
                           gui=gui,
                           record=record_video,
                           obstacles=obstacles
                           )
    else: #default
        env = CtrlAviary(drone_model=drone,
                         num_drones=num_drones,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=physics,
                         neighbourhood_radius=10,
                         freq=simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=gui,
                         record=record_video,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui
                         )##initialization of env, and connect to PyBullet

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()#no use for later

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(simulation_freq_hz/AGGR_PHY_STEPS),
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )#utils,duration sec?

    ###### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:#default
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]###control functions
    elif drone in [DroneModel.HB]:
        ctrl = [SimplePIDControl(drone_model=drone) for i in range(num_drones)]

    ###### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/control_freq_hz))###KEYWORD?
    print("CTRL_EVERY_N_STEPS",CTRL_EVERY_N_STEPS)
    print("env.SIM_FREQ",env.SIM_FREQ)
#  CTRL_EVERY_N_STEPS 5
#  env.SIM_FREQ 240
#control_freq_hz 48 
    
    
    action = {str(i): np.array([0,0,0,0]) for i in range(num_drones)}#DICTIONARY
    START = time.time()
    print("START Simulation time",START)
    ##########################DEBUG Visualization for straight line trajectory################################################
   # p.addUserDebugLine([0,0,0], [-NUM_WP*.005,-NUM_WP*.005,NUM_WP*.005], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    
    p.addUserDebugLine([0,0,0], [-1,-1,1], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    p.addUserDebugLine([-1,-1,1], [1,2,1.5], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    p.addUserDebugLine([1,2,1.5], [3,1,2], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    p.addUserDebugLine([3,1,2], [1,-1,2.5], lineColorRGB=[0, 0, 1], lifeTime=0, lineWidth=3)
    #visualisation
    for wp in range(0,len(TARGET_POS)-20,20):
        p.addUserDebugLine(TARGET_POS[wp], TARGET_POS[wp+20], lineColorRGB=[1, 0, 0], lifeTime=0, lineWidth=1)
    ######MINIMUM SNAP########
    for wp in range(0,len(TARGET_POS2)-20,20):
        p.addUserDebugLine(TARGET_POS2[wp], TARGET_POS2[wp+20], lineColorRGB=[0, 1, 0], lifeTime=0, lineWidth=1)
    ###########################################################################
    
    
    for i in range(0, int(duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):##duration_sec*env.SIM_FREQ, total time? AGGR_PHY_STEPS time step
    
        #### Make it rain rubber ducks #############################
        #if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)
        #print("env.step: obs", obs)
        #### Compute control at the desired frequency ##############
        if i%CTRL_EVERY_N_STEPS == 0:

            #### Compute control for the current way point #############
            for j in range(num_drones):
                action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                       state=obs[str(j)]["state"],
                                                                       #target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]])
                                                                       target_pos=np.hstack(TARGET_POS[wp_counters[j], 0:3])
                                                                       # target_pos=INIT_XYZS[j, :] + TARGET_POS[wp_counters[j], :],
                                                                       #target_rpy=INIT_RPYS[j, :]
                                                                       )#control 
            print("TARGET:",np.hstack(TARGET_POS[wp_counters[j], 0:3]))
            #### Go to the next way point and loop #####################
            for j in range(num_drones): 
                wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.SIM_FREQ,
                       state=obs[str(j)]["state"],
                       control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                       # control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
                       )

        #### Printout ##############################################
        if i%env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            if vision:
                for j in range(num_drones):
                    print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
                          obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),
                          obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"])
                          )

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.TIMESTEP)



    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()


#function run end

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=DEFAULT_VISION,      type=str2bool,      help='Whether to use VisionAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=DEFAULT_AGGREGATE,       type=str2bool,      help='Whether to aggregate physics steps (default: True)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))