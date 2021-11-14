from iktest import M, S
import sim # access all the VREP elements
import sys # for stopping at errors
import time # for time/sleeping
import numpy as np
import os
import modern_robotics as mr

#### Helper functions

def CalculateHomeAndScrewAxes():
    global M
    global S

    M = np.array([  [ 1,  0,  0,  -0.192],
                    [ 0,  1,  0,  0  ],
                    [ 0,  0,  1,  0.692 ],
                    [ 0,  0,  0,  1      ]])
    w1 = np.array([0,0,1])
    w2 = np.array([-1,0,0])
    w3 = np.array([-1,0,0])
    w4 = np.array([-1,0,0])
    w5 = np.array([0,0,1])
    w6 = np.array([-1,0,0])
    v1 = np.cross(-w1, np.array([0, 0, 0]))
    v2 = np.cross(-w2, np.array([0, 0, 0.152]))
    v3 = np.cross(-w3, np.array([0, 0, 0.396]))
    v4 = np.cross(-w4, np.array([0, 0, 0.609]))
    v5 = np.cross(-w5, np.array([-0.110, 0, 0]))
    v6 = np.cross(-w6, np.array([0, 0, 0.692]))

    S1 = np.concatenate((w1, v1), axis=None)
    S2 = np.concatenate((w2, v2), axis=None)
    S3 = np.concatenate((w3, v3), axis=None)
    S4 = np.concatenate((w4, v4), axis=None)
    S5 = np.concatenate((w5, v5), axis=None)
    S6 = np.concatenate((w6, v6), axis=None)

    S = np.array([S1, S2, S3, S4, S5, S6]).T

    return M, S

def GetJointAngles(handle_arr):
    # handles indexed from 1 to 6 for joints 1 to 6
    angles_arr = np.zeros(6) # indexed from 0 to 5 for joints 1 to 6

    for i in range(0, 6): # from 0 to 5 inclusive
        result, angles_arr[i] = sim.simxGetJointPosition(clientID, handle_arr[i+1], sim.simx_opmode_blocking)
        if result != sim.simx_return_ok:
            sys.exit('Unable to get angle for joint ' + str(i+1))
            
    return angles_arr

def SetJointAngles(handle_arr, joint_angles):
    # joint angles indexed from 0 to 5 for joints 1 to 6
    # handle_arr indexed from 1 to 6 for joints 1 to 6
    for i in range(0, 6):
        sim.simxSetJointTargetPosition(clientID, handle_arr[i+1], joint_angles[i], sim.simx_opmode_oneshot)
        time.sleep(0.5) # slight delay
    pass

def GetProxSensorDist(handle_arr):
    # 8th element of handle_arr is prox sensor
    prox_handle = handle_arr[8]
    result, detect_state, detected_point, detected_object_handle, detected_surace_norm_vec = sim.simxReadProximitySensor(clientID, prox_handle, sim.simx_opmode_buffer)
    if result!=sim.simx_return_ok:
        sys.exit('Unable to get prox sensor readings')
    return detect_state, detected_point

def VacuumGrip(on):
    sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", on, sim.simx_opmode_oneshot)

def CalculateFK(joint_angles):
    T_01 = mr.FKinSpace(M, S, joint_angles)
    return T_01

def MoveRobotUsingIK(position_list, handle_arr):
    print("Commanding robot to move to ", position_list)
    initial_theta_list = np.array([0,0,0,0,0,0])

    pose = np.array([   [0, -1, 0, position_list[0]],
                        [0, 0, -1, position_list[1]],
                        [1, 0, 0, position_list[2]],
                        [0, 0, 0, 1]])
    
    joint_angles, success = mr.IKinSpace(S, M, pose, initial_theta_list, 0.01, 0.001)
    
    print(success, joint_angles)
    print("Forward kinematics of calculated joint angles: ")
    print(CalculateFK(joint_angles))

    SetJointAngles(handle_arr, joint_angles)
    return joint_angles
####

#### Establish connections with coppelia sim

sim.simxFinish(-1) # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # start a connection

if (clientID != -1):
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

####

#### Get handles to all objects

handle_arr = [None] * 9 # here, 0 is base handle, 1-6 are joint handles, and 7 is end effector handle, and 8 is prox sensor

# Get "handle" to the base of robot
result, handle_arr[0] = sim.simxGetObjectHandle(clientID, 'UR3_link1_visible', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	sys.exit('Failed to get object handle for base frame')

joint_name = 'UR3_joint'
for i in range(1, 7): # from 1 to 6, inclusive
    cur_joint_name = joint_name + str(i)
    # Get handle of current joint
    result, handle_arr[i] = sim.simxGetObjectHandle(clientID, cur_joint_name, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
	    sys.exit('Failed to get object handle for ' + cur_joint_name)

# Get handle of end effector:
result, handle_arr[7] = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	sys.exit('Failed to get object handle for end effector')


# Get handle of proximity sensor
result, handle_arr[8] = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_1', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	sys.exit('Failed to get object handle for proximity sensor')

# attempt to do first reading to prepare next readings
result, detect_state, detected_point, detected_object_handle, detected_surace_norm_vec = sim.simxReadProximitySensor(clientID, handle_arr[8], sim.simx_opmode_streaming)

print("Sucessfully retrieved handles")

####

#### Start simulation

# enable the synchronous mode on the client:
sim.simxSynchronous(clientID, True)

# Start simulation
# Note not to start simulation, but only run simExtRemoteApiStart (19999) on every new coppelia sim window
print("Starting simulation")
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# Wait for things a bit
time.sleep(2)

####

#### Do things in simulation

# Calculate the home position and screw axes
CalculateHomeAndScrewAxes()

# Attempt to spawn sphere
# print("Spawning golf ball")
# sphere_path = os.path.abspath(os.getcwd()) + '/sphere_correct_loc.ttm'
# result, sphere_handle = sim.simxLoadModel(clientID, sphere_path, 0, sim.simx_opmode_oneshot_wait)
# if result != sim.simx_return_ok:
# 	sys.exit('Failed to get object handle for instantiated sphere')

print("Spawning box sample")
sphere_path = os.path.abspath(os.getcwd()) + '/box_sample.ttm'
result, sphere_handle = sim.simxLoadModel(clientID, sphere_path, 0, sim.simx_opmode_oneshot_wait)
if result != sim.simx_return_ok:
	sys.exit('Failed to get object handle for instantiated box')

# # Attempt to move UR3 to a set of joint angles
# print("Moving UR3 robot arm")
# desired_joint_angles = np.array([90*np.pi/180,-30*np.pi/180,-60*np.pi/180,0,90*np.pi/180,0])
# SetJointAngles(handle_arr, desired_joint_angles)
# delay to see position
# time.sleep(5)

detected_state, detected_point = GetProxSensorDist(handle_arr)
while (detected_state != 1):
    print("Object not detected")
    time.sleep(1)
    detected_state, detected_point = GetProxSensorDist(handle_arr)

print("Object detected")


result, sphere_position_list = sim.simxGetObjectPosition(clientID, sphere_handle, handle_arr[0], sim.simx_opmode_blocking)
print("Attempting to get pos relative to base of robot")
# print(sphere_position_list)
if result != sim.simx_return_ok:
	sys.exit('Failed to get object handle for instantiated sphere')
print("Object location with respect to base of robot is ", sphere_position_list)

sphere_position_list_pick_level = [sphere_position_list[0], sphere_position_list[1], sphere_position_list[2]+0.145]
sphere_position_list_above = [sphere_position_list[0], sphere_position_list[1], sphere_position_list[2] + 0.2]

MoveRobotUsingIK(sphere_position_list_above, handle_arr)
MoveRobotUsingIK(sphere_position_list_pick_level, handle_arr)

# print("Spawning box sample 2")
# sphere_path = os.path.abspath(os.getcwd()) + '/box_sample.ttm'
# result, sphere_handle = sim.simxLoadModel(clientID, sphere_path, 0, sim.simx_opmode_oneshot_wait)
# if result != sim.simx_return_ok:
# 	sys.exit('Failed to get object handle for instantiated box')

# turn on vacuum
VacuumGrip(1)
MoveRobotUsingIK(sphere_position_list_above, handle_arr)

dest_position_list_pick_up_level = [sphere_position_list[0]-0.2, sphere_position_list[1], sphere_position_list[2]+0.145]
dest_position_list_above = [sphere_position_list[0]-0.2, sphere_position_list[1]+0.1, sphere_position_list[2]+0.2]

MoveRobotUsingIK(dest_position_list_pick_up_level, handle_arr)

# turn off vacuum
VacuumGrip(0)
MoveRobotUsingIK(dest_position_list_above, handle_arr)

time.sleep(5)


# time.sleep(10)

####

#### Stop simulation

# delay a bit
time.sleep(2)

print("Stopping simulation")
# stop the simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)

# Close connection with Coppelia sim
# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
sim.simxGetPingTime(clientID)

# Now close the connection to CoppeliaSim:
print("Closing connection with remote server")
sim.simxFinish(clientID)

####