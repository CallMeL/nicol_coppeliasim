
#!/usr/bin/env python

start_coppelia = True
start_gazebo = False
use_platform = False

############################ Import #################################
if start_coppelia:
    import nicol_api
    import matplotlib.pyplot as plt
    from nicol_base import NicolJointPosition, NicolPose
    from nicol_cycleik import NicolCycleIK


else:
    from nias_action.core.nicol_cycleik import NicolCycleIK
    from nias_action.core.nicol_base import NicolJointPosition, NicolPose
    
import random
import numpy as np
import random
import time
import pathlib

import cv2 as cv

import matplotlib.pyplot as plt
import argparse
############################ local functions #################################
parser = argparse.ArgumentParser()
parser.add_argument("-s","--Start")
args = parser.parse_args()
#sample random pose from reachable space
def get_random_pose(left=False):
    if left:
        x = random.uniform(0.35, 0.75)
        y = random.uniform(0.2, 0.7)
        z = random.uniform(0.9, 1.2)
    else:
        x = random.uniform(0.35, 0.75)
        y = random.uniform(-0.2, -0.7)
        z = random.uniform(0.9, 1.2)

    if left:
        return NicolPose(position=[x, y, z], orientation=[np.pi/4, -np.pi/4, 0., 0.])
    else:
        return NicolPose(position=[x, y, z], orientation=[0., 0., np.pi/4, np.pi/4])

#execute a trajectory in joint space
def execute_js_trajectory(js_array, trajectory):
    for k in range(len(js_array)):
        head.set_pose_target(right.get_eef_pose())
        js = js_array[k]
        if not start_coppelia:
            fk_pose = right.calc_fk(NicolJointPosition(js))
            print("\n\nHere\n\n")
        js_reachable = right.set_joint_position(js)


def calculate_trajectory(origin, target, points_per_trajectory):
    delta_x = (target.position.x - origin.position.x) / (points_per_trajectory - 1)
    delta_y = (target.position.y - origin.position.y) / (points_per_trajectory - 1)
    delta_z = (target.position.z - origin.position.z) / (points_per_trajectory - 1)
    trajectory = []
    current_point = [0., 0., 0.]
    for i in range(points_per_trajectory):
        current_point[0] = origin.position.x + (i * delta_x)
        current_point[1] = origin.position.y + (i * delta_y)
        current_point[2] = origin.position.z + (i * delta_z)
        #print(np.array(current_point))
        #print(np.array(orientation))
        trajectory.append(NicolPose(position=current_point, orientation=[origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w]))
    return trajectory

def point_at(point):
    point = np.array(point)
    eef_point = point.copy()
    eef_point -= np.array([0.165, 0.05,0])
    eef_point[2] = 0.95

    orientation = [0.21738 , 0.24031, 0.62923 ,0.70645]
    target_pose = NicolPose(eef_point, orientation)
    return target_pose

############################ Initialize #################################
nicol = None
if start_coppelia:
    nicol = NicolCycleIK(scene="./nicol_hri.ttt",start_scene=args.Start,talker=False)
else:
    #nicol = NicolCycleIK(scene="./nicol_hri.ttt",start_scene=False,talker=False)
    if use_platform:
        nicol = NicolCycleIK(launch_nicol=False, sleep_timeout=0.003)
    else:
        nicol = NicolCycleIK(launch_nicol=False, service_timeout=40)


head  = nicol.head()    # 3D-Printed head structure
left  = nicol.left()    # Left  OpenManipulator + RH8D
right = nicol.right()   # Right OpenManipulator + RH8D

############################ Set base Pose ###########################

'''
if not start_coppelia:
    right.set_joint_position_for_hand([-np.pi] * 5,block=True)

import pandas as pd
arms = []
for i in range(0,10000):
    nicol.start_simulation()
    head.set_joint_position([-1.1,0],block=True)
    right.set_joint_position([1.57] + [0.] * 7,block=True)
    left.set_joint_position([-1.57] + [0.] * 7,block=True)
    pose = get_random_pose()
    right.set_pose_target(pose)
    rightArm = right.get_joint_position().position
    print("right: ",rightArm)
    pose = get_random_pose(True)
    left.set_pose_target(pose)
    leftArm = left.get_joint_position().position
    #frame = nicol.get_opposite_camera_img(persistent=True,prefix=str(i))
    #frame2 = nicol.get_left_eye_camera_img(persistent=True,prefix=str(i))
    arms.append([leftArm,rightArm])
    df = pd.DataFrame(arms)
    #df.to_csv("./generatedDataWithArmsNew/frame_rgb/armsnew.csv",header=["right","left"],index=None)
    nicol.stop_simulation()
    time.sleep(0.1)

'''

if not start_coppelia:
    right.set_joint_position_for_hand([-np.pi] * 5,block=True)

import pandas as pd
arms = []
for i in range(0,10):
    nicol.start_simulation()
    head.set_joint_position([-1.1,0],block=True)
    right.set_joint_position([1.57] + [0.] * 7,block=True)
    left.set_joint_position([-1.57] + [0.] * 7,block=True)
    pose = get_random_pose()
    right.set_pose_target(pose)
    rightArm = right.get_joint_position().position
    print("right: ",rightArm)
    pose = get_random_pose(True)
    left.set_pose_target(pose)
    leftArm = left.get_joint_position().position
    #frame = nicol.get_opposite_camera_img(persistent=True,prefix=str(i))
    #frame2 = nicol.get_left_eye_camera_img(persistent=True,prefix=str(i))
    arms.append([leftArm,rightArm])
    df = pd.DataFrame(arms)
    df.to_csv("./exportData/armsnew.csv",header=["right","left"],index=None)
    nicol.stop_simulation()
    time.sleep(0.1)

frame2 = nicol.get_left_eye_camera_img(persistent = True, save_path = "./exportData")