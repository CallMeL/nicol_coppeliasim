# Copyright (c) 2023. Lennart Clasmeier. GNU General Public License https://www.gnu.org/licenses/gpl-3.0.html.

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
    try:
        from dependencies.nicol_tts.nicol_speech import NICOL_TALKER

    except:
        pass

else:
    from nias_action.core.nicol_cycleik import NicolCycleIK
    from nias_action.core.nicol_base import NicolJointPosition, NicolPose
    
import random
import numpy as np
import random
import time
import pathlib

import cv2 as cv

############################ local functions #################################

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
    nicol = NicolCycleIK(scene="./nicol_hri.ttt",start_scene=True,talker=True)
    # participant = Participant("p225")
else:
    if use_platform:
        nicol = NicolCycleIK(launch_nicol=False, sleep_timeout=0.003)
    else:
        nicol = NicolCycleIK(launch_nicol=False, service_timeout=40)


head  = nicol.head()    # 3D-Printed head structure
left  = nicol.left()    # Left  OpenManipulator + RH8D
right = nicol.right()   # Right OpenManipulator + RH8D

############################ Set base Pose ###########################

head.set_joint_position([0,0],block=True)
right.set_joint_position([1.57] + [0.] * 7,block=True)
left.set_joint_position([-1.57] + [0.] * 7,block=True)
if not start_coppelia:
    right.set_joint_position_for_hand([-np.pi] * 5,block=True)

#right.close_hand(0.85)
#print(f"joint_position: {right.get_joint_position()}")

#right.set_joint_position_for_hand([np.pi] * 5,block=True)

#print(f"joint_position: {right.get_joint_position()}")

#right.close_hand(0.85)

#time.sleep(5)
############################ Get Picture ###########################
left_eye_image = nicol.get_left_eye_camera_img(persistent = True, save_path = "./exportData")
import matplotlib.pyplot as plt
import cv2

# Convert the image to RGB if necessary
left_eye_image_rgb = cv2.cvtColor(left_eye_image, cv2.COLOR_BGR2RGB)
plt.imshow(left_eye_image_rgb)
plt.title("Left Eye Camera Image")
plt.show()
############################ HRI Stuff #######################################

block_positions = {
    "red": [0.6,-0.52,0.86],
    "green": [0.6, -0.375,0.825],
    "yellow": [0.6, -0.225,0.825],
    "blue": [0.6, -0.075,0.825],
    "orange":[0.6, 0.075,0.825]
    }

if start_coppelia:
    for name in block_positions.keys():
        sim = nicol.sim
        target_handle=sim.getObject("/"+name)
        position = block_positions[name]
        if name == "red":
            position[2] = 0.91
        sim.setObjectPosition(target_handle,sim.handle_world,position)
        nicol.step_simulation(1)
else:
    input("please place the objects on the table")


right.set_joint_position([np.pi/2,0,0,0,0,0,0,0])

x = block_positions["blue"][0] - 0.15
y = block_positions["blue"][1] - 0.05
z = 0.90

head.set_pose_target(NicolPose(block_positions["blue"],[0,0,0,0]))

right.set_pose_target(NicolPose([x,y,z+0.2],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x,y,z+0.1],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x,y,z],[0,0,np.pi/4, np.pi/4]))

right.close_hand(0)
if start_coppelia:
    nicol.step_simulation(100)

right.set_pose_target(NicolPose([x - 0.0,y-0.02,z+0.01],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x - 0.0,y-0.05,z+0.01],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x - 0.05,y-0.1,z+0.025],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x - 0.05,y-0.2,z+0.1],[0,0,np.pi/4, np.pi/4]))
right.set_pose_target(NicolPose([x + 0.1,y-0.3,z+0.2],[0,0,np.pi/4, np.pi/4]))

right.set_joint_position([np.pi/2])
right.set_joint_position([np.pi/2]+[0]*7)

head.set_joint_position(NicolJointPosition([0.0, 0.0]))
right.set_joint_position([1.57, 0., 0., 0., 0., 0., 0., 0.])

#reset right arm in safe position
right.set_joint_position([0.31, 0.2, -0.2, 1.5, 1.28, 0.0, 0.0, 0.0])


if start_coppelia:
    nicol.stop_simulation()
    nicol.stop_coppelia()