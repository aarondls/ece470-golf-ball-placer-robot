import sim # access all the VREP elements
import sys # for stopping at errors
import time # for time/sleeping
import numpy as np
import os

#### Helper functions

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

# Attempt to spawn sphere
print("Spawning golf ball")
sphere_path = os.path.abspath(os.getcwd()) + '/sphere_sample.ttm'
result, sphere_handle = sim.simxLoadModel(clientID, sphere_path, 0, sim.simx_opmode_oneshot_wait)
if result != sim.simx_return_ok:
	sys.exit('Failed to get object handle for instantiated sphere')

# Attempt to move UR3 to a set of joint angles
print("Moving UR3 robot arm")
desired_joint_angles = np.array([0,0,-0.5*np.pi,0.5*np.pi,-0.5*np.pi,-0.5*np.pi])
SetJointAngles(handle_arr, desired_joint_angles)

# delay to see position
time.sleep(2)

# attempt to read prox sensor
print('Attempting to read prox sensor')
print("Boolean represents if object is detected, and the three value array are the xyz coordinates of detected object")
for i in range(0, 10):
    detected_state, detected_point = GetProxSensorDist(handle_arr)
    print(detected_state, detected_point)
    time.sleep(1)

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