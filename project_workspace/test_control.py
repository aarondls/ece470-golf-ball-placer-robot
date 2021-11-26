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
        # time.sleep(0.5) # slight delay
    pass

def GetProxSensorDist(prox_handle):
    result, detect_state, detected_point, detected_object_handle, detected_surace_norm_vec = sim.simxReadProximitySensor(clientID, prox_handle, sim.simx_opmode_buffer)
    if result!=sim.simx_return_ok:
        sys.exit('Unable to get prox sensor readings')
    return detect_state, detected_point

def VacuumGrip(on):
    sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", on, sim.simx_opmode_oneshot)

def CalculateFK(joint_angles):
    T_01 = mr.FKinSpace(M, S, joint_angles)
    return T_01

def CalculateJointConfiguration(desired_pose, theta_guess, handle_arr):
    current_joint_angles = GetJointAngles(handle_arr)

    joint_angles, success = mr.IKinSpace(S, M, desired_pose, theta_guess, 0.01, 0.001)

    if not(success):
        print("No solution found")
        joint_angles = current_joint_angles
        
    # print(success, joint_angles)
    # print("Forward kinematics of calculated joint angles: ")
    # print(CalculateFK(joint_angles))

    return joint_angles

# takes desired pose and moves robot to pose
# uses current joint angle as initial guess
def MoveToPoseUsingIK(desired_pose, handle_arr):
    print("Commanding robot to move to ", desired_pose)
    initial_theta_list_raw = GetJointAngles(handle_arr)

    joint_angles, success = mr.IKinSpace(S, M, desired_pose, initial_theta_list_raw, 0.01, 0.001)

    if not(success):
        print("No solution found")

    SetJointAngles(handle_arr, joint_angles)

    return joint_angles



def MoveStraightCartessian(desired_position_list, total_time, handle_arr):
    print("Commanding robot to move to ", desired_position_list)

    # get current position and put it in SE(3) form
    current_joint_angles = GetJointAngles(handle_arr)
    X_start = CalculateFK(current_joint_angles)

    X_desired = np.array([[0, -1, 0, desired_position_list[0]],
                        [0, 0, -1, desired_position_list[1]],
                        [1, 0, 0, desired_position_list[2]],
                        [0, 0, 0, 1]])

    # elapsed time between each matrix turns out to total time / (total matrices count - 1)
    elapsed_time = 0.1
    total_matrix_count = (total_time / elapsed_time) + 1
    straight_line_traj = mr.CartesianTrajectory(X_start, X_desired, total_time, total_matrix_count, 5)

    for current_se3_pose in straight_line_traj:
        MoveToPoseUsingIK(current_se3_pose, handle_arr)
        # time.sleep(0.1)
        # no need sleep since calculation takes a while

def MoveStraightJointSpace(desired_pose, total_time, handle_arr):
    print("Commanding robot to move to ", desired_pose)

    # get current position and put it in SE(3) form
    current_joint_angles = GetJointAngles(handle_arr)

    # calculate desired joint angles given desired pose
    desired_joint_angles = CalculateJointConfiguration(desired_pose, [0,0,0,0,0,0], handle_arr)

    # elapsed time between each matrix turns out to total time / (total matrices count - 1)
    elapsed_time = 0.1
    total_matrix_count = (total_time / elapsed_time) + 1
    straight_line_traj = mr.JointTrajectory(current_joint_angles, desired_joint_angles, total_time, total_matrix_count, 5)

    for current_joint_configuration in straight_line_traj:
        SetJointAngles(handle_arr, current_joint_configuration)
        time.sleep(elapsed_time)


def SpawmDynamicGolfBall(clientID):
    print("Spawning golf ball")

    sphere_path = os.path.abspath(os.getcwd()) + '/sphere_correct_loc.ttm'
    result, sphere_handle = sim.simxLoadModel(clientID, sphere_path, 0, sim.simx_opmode_oneshot_wait)
    if result != sim.simx_return_ok:
        sys.exit('Failed to get object handle for instantiated sphere')
    
    return sphere_handle

def SpawnGolfTee(clientID):
    print("Spawning golf tee")

    tee_path = os.path.abspath(os.getcwd()) + '/golf_tee_correct_loc.ttm'
    result, tee_handle = sim.simxLoadModel(clientID, tee_path, 0, sim.simx_opmode_oneshot_wait)
    if result != sim.simx_return_ok:
        sys.exit('Failed to get object handle for instantiated golf tee')

    return tee_handle

def WaitForGolfBallDetection(prox_handle):
    detected_state, detected_point = GetProxSensorDist(prox_handle)
    while (detected_state != 1):
        print("Object not detected")
        time.sleep(1)
        detected_state, detected_point = GetProxSensorDist(prox_handle)

    print("Object detected")

# simulate the ball arriving, being moved to the tee, and being hit from the tee
def SimulateShotCycle(clientID, handle_arr):

    # Attempt to spawn golf ball
    golf_ball_handle = SpawmDynamicGolfBall(clientID)
    tee_handle = handle_arr[9]

    # Don't proceed until the ball is detected by the sensor
    WaitForGolfBallDetection(handle_arr[8])


    result, golf_ball_position_list = sim.simxGetObjectPosition(clientID, golf_ball_handle, handle_arr[0], sim.simx_opmode_blocking)
    print("Attempting to get pos of golf ball relative to base of robot")
    if result != sim.simx_return_ok:
        sys.exit('Failed to get object position for instantiated golf ball')
    print("Object location with respect to base of robot is ", golf_ball_position_list)

    golf_ball_position_list_pick_level = [golf_ball_position_list[0], golf_ball_position_list[1], golf_ball_position_list[2]+0.146]
    golf_ball_position_list_above = [golf_ball_position_list[0], golf_ball_position_list[1], golf_ball_position_list[2] + 0.2]

    MoveStraightCartessian(golf_ball_position_list_above, 1, handle_arr)
    MoveStraightCartessian(golf_ball_position_list_pick_level, 1, handle_arr)

    # turn on vacuum
    VacuumGrip(1)
    MoveStraightCartessian(golf_ball_position_list_above, 1, handle_arr)

    result, tee_position_list = sim.simxGetObjectPosition(clientID, tee_handle, handle_arr[0], sim.simx_opmode_blocking)
    print("Attempting to get pos of golf tee to base of robot")
    if result != sim.simx_return_ok:
        sys.exit('Failed to get object position for instantiated golf tee')
    print("Tee location with respect to base of robot is ", tee_position_list)

    dest_position_list_pick_up_level = [tee_position_list[0], tee_position_list[1], tee_position_list[2]+0.225]
    dest_position_list_above = [tee_position_list[0], tee_position_list[1], tee_position_list[2]+0.25]

    MoveStraightCartessian(dest_position_list_above, 2, handle_arr)
    MoveStraightCartessian(dest_position_list_pick_up_level, 1, handle_arr)

    # turn off vacuum
    VacuumGrip(0)

    # time.sleep(0.018)
    time.sleep(0.03)

    print("Instantiating static sphere")
    static_sphere_path = os.path.abspath(os.getcwd()) + '/golf_ball_correct_loc_static.ttm'

    sim.simxRemoveModel(clientID, golf_ball_handle, sim.simx_opmode_oneshot_wait)

    result, static_sphere_handle = sim.simxLoadModel(clientID, static_sphere_path, 0, sim.simx_opmode_oneshot_wait)
    if result != sim.simx_return_ok:
        sys.exit('Failed to get object handle for instantiated static sphere')

    MoveStraightCartessian(dest_position_list_above, 1, handle_arr)

    MoveStraightCartessian([0,-0.3,0.4], 1, handle_arr)

    # hit the golf ball (delete it)
    sim.simxRemoveModel(clientID, static_sphere_handle, sim.simx_opmode_oneshot_wait)

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

handle_arr = [None] * 10 # here, 0 is base handle, 1-6 are joint handles, and 7 is end effector handle, and 8 is prox sensor, 9 is golf ball

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

# Calculate the home position and screw axes
CalculateHomeAndScrewAxes()

# move to start position
SetJointAngles(handle_arr, [0,0,0,0,0,0])
MoveStraightJointSpace(np.array([[0, -1, 0, 0],
                     [0, 0, -1, -0.3],
                     [1, 0, 0, 0.4],
                     [0, 0, 0, 1]]), 1, handle_arr)


time.sleep(2)

####

#### Do things in simulation

# Attempt to spawn golf tee
handle_arr[9] = SpawnGolfTee(clientID)

# run single shot cycle
SimulateShotCycle(clientID, handle_arr)

# run shot cycle 5 times
for i in range(5):
    SimulateShotCycle(clientID, handle_arr)

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