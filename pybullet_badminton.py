# Course Project: Badminton robot
# Robot Learning, ME555.09 Fall 23'
# Haobo Yuan
# Acknowledgement: The robot model is Fetch robot, downloaded from https://github.com/ZebraDevs/fetch_ros/tree/melodic-devel/fetch_description
#                  I do not claim copyright for any model files under this repo.
# Description: This is a simulation for a simplified badminton robot. 
#              The robot could move along a straight line through the keys "a"/"d" on keyboard, and control its arm to hit the shuttlecock through sliders manually.

import os
import pybullet as p
import pybullet_data
import numpy as np
import time

clear_screen = lambda: os.system('cls' if os.name == 'nt' else 'clear')  # suits for windows and linux
clear_screen()

#############################
##    Randering config     ##
#############################
use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)  # disable first to speed up the setup, and enable it later
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1) 
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)  # Disable integrated graphics on CPU
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)


# # 1. Badminton court: 13.4 x 6.1 x 0.05 meters
court_length = 13.4
court_width = 6.1
court_height = 0.05
court_id = p.loadURDF("data/badminton_court.urdf", [0, 0, -court_height/2], useFixedBase=True)
'''
------------length-----------
|             y             |
|             ^        ^    |
|             |        |    |
Width         O-->x  Robot  |
|                           |
|                           |
|                           |
-------------net-------------
'''

# Add a plane as the floor of the court, better display for a outside shuttlecock
plane_id = p.loadURDF("plane.urdf", [0, 0, -court_height], useMaximalCoordinates=True, useFixedBase=True)  # useMaximalCoordinates=True improves the performance for object without any joints

# Build a wall as the net's collision object
wall_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.05, 6.1/2, 1.55/2])  # The wall is 0.05 x 6.1 x 1.55 meters
    # the thickness 0.05 makes the wall stand well, otherwise it will fall down
    # width/2 and height/2 because the func() is halfExtents, not full extents
wall_collison_box_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.05, 6.1/2, 1.55/2])
wall_id = p.createMultiBody(
    baseMass=10000,
    baseCollisionShapeIndex=wall_collison_box_id,
    baseVisualShapeIndex=wall_visual_shape_id,
    basePosition=[0, 0, 1.55/2])

# # 2. Shuttlecock
# shuttlecock_id = p.loadURDF("sphere_small.urdf", [-court_length/4, 0, 0.4], useMaximalCoordinates=True)
scale = [0.05, 0.05, 0.05]
ball_start_pos = [-court_length/4, 0.5, 0.4]
ball_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="data/shuttlecock/11744_Shuttlecock_v1_l3.obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.4, 0.4, 0],
    meshScale=scale)
ball_collison_box_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="data/shuttlecock/11744_Shuttlecock_v1_l3.obj",
    meshScale=scale)
shuttlecock_id = p.createMultiBody(
    baseMass=0.005,
    baseCollisionShapeIndex=ball_collison_box_id,
    baseVisualShapeIndex=ball_visual_shape_id,
    basePosition=ball_start_pos,
    baseOrientation=p.getQuaternionFromEuler([0, -2*np.pi/3, 0]),
    )
ball_init_linear_vel = [5, 0, 10]
ball_init_angular_vel = [0, 0, 0]
p.resetBaseVelocity(shuttlecock_id, ball_init_linear_vel, ball_init_angular_vel)

# # 3. Robot
robot_start_pos = [court_length/4, 0, -court_height/2]
robot_start_orn = p.getQuaternionFromEuler([0, 0, np.pi/2])  # counter-clockwise is positive
robot_base_pos, robot_base_orn = robot_start_pos, robot_start_orn  # init the updating variables
robot_id = p.loadURDF("data/robots/fetch.urdf", robot_start_pos, robot_start_orn)
end_effector_link_index = 18  # Used for Inv Kinematics

# Find unfixed joints' indexes
available_joints_indexes = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
# Classify joints' indexes
wheel_joints_indexes = [1,0]  # Note: 1: left wheel, 0: right wheel
arm_joints_indexes = [10,11,12,13,14,15,16]
once_joints_indexes = [3]
unused_joints_indexes = [2,4,18,19,20]

# The model is not precise, having different sizes of wheels. To make up for this, always use left_wheel_velo * correction_factor
# Because of imprecision, the bottom of the 6 wheels (2 Driven + 4 Castor) are not on the same plane, so the robot always has a small rotation on yaw-axis
# To make up, the robot's locomotion on x-axis and orientation on yaw-axis are reseted enforcedly for several simulated steps
correction_factor = 0.95

# Enforce the robot to be stable and stay on y-axis
def enforce_stable():
    p.resetBasePositionAndOrientation(robot_id, 
                                      [robot_start_pos[0], robot_base_pos[1], robot_base_pos[2]], 
                                      robot_start_orn)  # restrict the robot's x&rpy axis, let it has only 2 DOF


#############################
##      UserDebugParam     ##
#############################


# A debug line to show robot's boundary
def myDebugLine():
    froms = [[court_length/4, -court_width/2, 0]]
    tos = [[court_length/4, court_width/2, 0]]
    for f, t in zip(froms, tos):
        p.addUserDebugLine(lineFromXYZ=f, lineToXYZ=t, lineColorRGB=[1, 0, 0], lineWidth=2)
myDebugLine()

# Camera info for preset distance and angles
cam_global_info = [7, 30, -45, [(1/2)*court_length/4, 0, 0]]
cam_robotback_info = [3, 90, -20, robot_start_pos]
camera_info = cam_global_info
def myResetCamera(cam_info):  # cam_info = [cam_distance, cam_yaw, cam_pitch, cam_pos]
    p.resetDebugVisualizerCamera(cameraDistance=cam_info[0], cameraYaw=cam_info[1], cameraPitch=cam_info[2], cameraTargetPosition=cam_info[3])

# Get (index, name) for UserDebugParam for better display
motor_link_tuples = [(p.getJointInfo(robot_id, i)[0], str(p.getJointInfo(robot_id, i)[1]))   # 0:index 1:name
    for i in arm_joints_indexes]

# # 1 wheels' together control, now changed to keyboard control

# # 2 arm's  control
motor_position_range=[-np.pi, np.pi]
# set arm's j1 and j3's init position as (-1 in rad), others as 0, to fold the arm
arm_start_positions = [0 if i not in [1, 3] else -1 for i in range(len(arm_joints_indexes))]
for i, joint_id in enumerate(arm_joints_indexes):
    p.resetJointState(robot_id, joint_id, arm_start_positions[i])
# add 7 sliders for 7 arm joints' position control
motor_position_params_ids = [p.addUserDebugParameter(
    paramName=motor_link_tuples[i][1] + " Position",
    rangeMin=motor_position_range[0],
    rangeMax=motor_position_range[1],
    startValue=arm_start_positions[i]
) for i in range(len(arm_joints_indexes))]

# # 3 Reset button
reset_button_id = p.addUserDebugParameter("Reset", 1, 0, 0)


#######################
##    Simulation     ##
#######################

clear_screen()

# Randering config again
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # Here is the time to enable the rendering
# Record video
# recorder = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "log/robotmove.mp4")

step_counter = 0
step_counter_reset_btn = 0
step_counter_collision = 0
time_step = 1/240
end_counter = 1e5

wheel_vel = 0
wheel_max_vel = 500
wheel_force = [10] * 2 

p.setRealTimeSimulation(1)
while True:
    step_counter += 1
    keys = p.getKeyboardEvents()

    # # 1. Control the robot
    robot_base_pos, robot_base_orn = p.getBasePositionAndOrientation(robot_id)  # Get the position and orientation of the base of the robot

    # 1.1 Lock unused joints
    p.setJointMotorControlArray(robot_id, once_joints_indexes, p.POSITION_CONTROL, targetPositions = [np.pi], forces=[100000]*len(once_joints_indexes))
    p.setJointMotorControlArray(robot_id, unused_joints_indexes, p.POSITION_CONTROL, forces=[100000]*len(unused_joints_indexes))
    
    # 1.2 Control the wheels through keyboard
    if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        wheel_vel = -wheel_max_vel
    elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        wheel_vel = wheel_max_vel
    # if not, set vel to 0
    elif ord('a') in keys and keys[ord('a')] & p.KEY_WAS_RELEASED:
        wheel_vel = 0
    elif ord('d') in keys and keys[ord('d')] & p.KEY_WAS_RELEASED:
        wheel_vel = 0

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=wheel_joints_indexes,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[wheel_vel*correction_factor, wheel_vel],  # make up for the model's imprecise wheels
        forces=wheel_force 
    )

    # 1.3 Control the arm, also init the arm's position through UserDebugParam
    positions = [p.readUserDebugParameter(param_id) for param_id in motor_position_params_ids]
    for joint_index, position in zip([i for i, _ in motor_link_tuples], positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,
            force=100
        )

    # 1.4 Enforce the robot to move along y-axis because the model is not precise, having different sizes of wheels
    if step_counter % 60 == 0:  # enforcement applied frequency, less than 1s
        enforce_stable()

    # 1.5 TODO: Mouse control for arm
    # mouse = p.getMouseEvents()
    # if len(mouse):
    #     print(mouse)

    # Inv Kinematics for the arm
    # End effector's target position and orientation
    target_pos = [0.1, 0, 0.15]
    target_orn = p.getQuaternionFromEuler([0, 0, 3*np.pi/2])
    ik_joint_positions = p.calculateInverseKinematics(robot_id, end_effector_link_index, target_pos, target_orn)
    # if i%(0.5/time_step) == 0:
    #     print('\n\nInv Kinematics Results: ', ik_joint_positions)

    # Implement the Inv Kinematics
    # for i, joint_position in enumerate(ik_joint_positions):
    #     p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=joint_position, force=500)


    # # 2. Camera Setting
    # 2.1 GUI rander camera
    myResetCamera(camera_info) if step_counter<100 else None  # set at the beginning
    # 2.2 Synthetic camera
    rpy = p.getEulerFromQuaternion(robot_base_orn),  # Notice! unit is rad
    rpy = np.rad2deg(rpy[0])
    viewMatrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[robot_base_pos[0] - court_length/4, robot_base_pos[1], robot_base_pos[2]+0.4],
        distance=court_length/4,
        yaw=rpy[2],  # Notice! unit is degree
        pitch=rpy[1],
        roll=rpy[0],
        upAxisIndex=2  # "2" for z-axis is up
    )
    projectionMatrix = p.computeProjectionMatrixFOV(
        fov=90,
        aspect=1.5,  # 1.5 is the ratio of width and height of the camera
        nearVal=0.01,
        farVal=100.0
    )
    p.getCameraImage(480, 320, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)


    # # 3 Reset button
    if p.readUserDebugParameter(reset_button_id) == 1:
        
        # 3.1 Reset sliders and button
        # Delete the old sliders and button
        p.removeAllUserParameters()
        # Rebuild the sliders and button
        start_values = [0 if i not in [1, 3] else -1 for i in range(len(arm_joints_indexes))]  # set arm's init position in rad to fold the arm
        motor_position_params_ids = [p.addUserDebugParameter(
            paramName=motor_link_tuples[i][1] + " Position",
            rangeMin=motor_position_range[0],
            rangeMax=motor_position_range[1],
            startValue=start_values[i]
        ) for i in range(len(arm_joints_indexes))]
        reset_button_id = p.addUserDebugParameter("Reset", 1, 0, 0)
        
        # 3.2 Reset robot' arm to init position 
        for i, joint_id in enumerate(arm_joints_indexes):
            p.resetJointState(robot_id, joint_id, arm_start_positions[i])
        
        # 3.3 Enforce the robot to be stable for a few attempts
        step_counter_reset_btn = step_counter

        # 3.4 Reset camera
        myResetCamera(camera_info)

        # 3.5 Reset shuttlecock
        p.resetBasePositionAndOrientation(shuttlecock_id, [-court_length/4, 0, 0.4], [0, 0, 0, 1])
        p.resetBaseVelocity(shuttlecock_id, ball_init_linear_vel, ball_init_angular_vel)

        # 3.6 Clean screen
        clear_screen()
    # following part of 3.3
    if step_counter - step_counter_reset_btn < (1/4)/time_step:  
        enforce_stable()


    # # 4. Collision detection
    # Check if the robot collide the shuttlecock
    P_min, P_max = p.getAABB(shuttlecock_id)
    id_tuple = p.getOverlappingObjects(P_min, P_max)
    if len(id_tuple) > 1:  # >1 means collision happens
        for ID, _ in id_tuple:
            if ID == robot_id:  # check if the object the shuttlecock collided is robot
                print(f"\n\n! Robot Hits Shuttlecock!, the step is {step_counter}")
                step_counter_collision = step_counter
                p.addUserDebugText(text=" !Robot Hits Shuttlecock!", textPosition=[0, 0, 1], textColorRGB=[1, 0, 0], textSize=2)  # Notice: repeatly triggered will declare many text_id
    # delete all the texts after a short period
    if step_counter - step_counter_collision == (1/2)/time_step:  
        p.removeAllUserDebugItems()  # remove all debug items (texts + lines)
        myDebugLine()  # redraw the debug line

    # # 5. Exit the program if forget to stop manually
    if step_counter > 0.99 * end_counter:
        p.addUserDebugText(text="Simulation stop soon!", textPosition=[0, 0, 1], textColorRGB=[1, 0, 0], textSize=2)
    if step_counter > end_counter:
        print("It seems that I forget to stop the silumation. Stop and give my laptop a rest.")
        break

    time.sleep(time_step)

# Stop recording
# p.stopStateLogging(recorder)
p.disconnect()
