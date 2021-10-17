# ece470-golf-ball-placer-robot

Repository for the ECE 470 Final Project

## Project Update 1 - 10/16/2021

Code for project update 1 is at the [update 1](/update_1) folder.
A [demonstration video](https://youtu.be/9_1yFtfyQoE) shows the current functionalities.

### Functionalities

The sample_scene.ttt CoppeliaSim scene file has a UR3 robot arm, a proximity laser sensor, a white sphere representing a golf ball, and a potted plant. The potted plant can be dragged around to demonstrate that the proximity sensor can detect it.

The Python script named test_control.py is then able to communicate with CoppeliaSim and control the simulation. This script handles starting and stopping the simulation, as well as three main functionalities of interest that is integral to the project.

#### Controlling the UR3 and receiving joint feedback

The script is able to control the joint positions of the UR3 robot. For this, the helper function `SetJointAngles` is defined at line 20.

The usage of this function is demonstrated by line 108, which moves the UR3 to the specific joint angles at line 113.

The script is also able to read the current joint positions of the UR3. This is achieved by the helper function `GetJointAngles` defined at line 9.

#### Reading distance measurements from proximity sensor

The data from the laser proximity sensor is read using the defined `GetProxSensorDist` helper function at line 28. This function returns whether there is an object detected by the sensor and its detected coordinates.

The usage of this function is demonstrated by the loop in line 121. The pair of sensor readings are then printed out to the console in 1 second intervals, for 15 iterations.

#### Spawning a new golf ball

A new sphere is spawned by the script at line 106, which represents the golf ball.

This functionality is necessary as it is not practical to drag and drop the golf ball model while the simulation is playing. The script handles spawning a new golf ball, which allows simulating new golf balls ariving at the pick-up location.

### Usage

Launch the CoppeliaSim

```Bash
./coppeliaSim.sh
```

then open the sample_scene.ttt scene. On the bottom command line of CoppeliaSim, run the command

```Lua
simExtRemoteApiStart(19999)
```

to start the server used by the Python script to communicate with the simulation.

The simulation is then controlled by the test_control.py script. To start the simulation, run this Python script.

```Bash
python test_control.py 
```

This script will print out status updates of what the script is currently doing, and will exit with a descriptive message if an error is encountered.
