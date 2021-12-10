# ece470-golf-ball-placer-robot

Repository for the ECE 470 Final Project

# Final Update - 12/10/21

The project is fully implemented in the CoppeliaSim simulator, with a Python script controlling the entire simulation via the remote API. All the necessary function implementations are defined within this main Python script.

In this simulation, a UR3 is used as the robot arm, while a sphere is used to represent the golf ball. To calculate the pose of the UR3 to achieve desired end-effector configurations, a numeric inverse kinematics solver based on the Newton-Raphson method is used.

The trajectory between desired end-effector configurations is then calculated with either a straight-line trajectory in joint space or Cartesian space. Cubic polynomial time scaling was initially used, but the movement was jerky. Quintic polynomial time scaling was then used to constrain the jerk to be 0 at both endpoints of the movement. Finally, for faster movement of the robot based on its joint velocity and acceleration limits, trapezoidal time scaling in joint space is also implemented.

![Full motion](/Images/full_motion.gif)

Code for this final update is at the [final update](/final_update) folder. Usage is still similar as in project update 1 and 2. Running the golf_sim.py script controls the simulation.

# Project Update 2 - 11/13/21

Code for project update 2 is at the [update 2](/update_2) folder.
A [demonstration video](https://youtu.be/sZv9fij2xtg) shows the current functionalities.

## Functionalities

This update builds on the same scene presented in the first project update. The scene now spawns a block to demonstrate the pick and place functionality of the robot. In addition, the location of the proximity sensor is moved closer to the robot to detect when and where the block is spawned.

The test_control.py script is now able to pick and place a detected object by the proximity sensor. These functionalities are detailed below.

### Detecting the location of the desired object to pick up

The script uses the proximity sensor to detect when the block is spawned, then gets the position of the block with respect to the base of the UR3.

It is important for this position to be in the base frame of the UR3 in order for the implemented forward and inverse kinematics to return the correct calculations.

### Calculating the joint angles to pick up the desired object using inverse kinematics

To calculate the joint angles for the UR3 to reach the position of the block, inverse kinematics is used. For this, the home configuration and screw axes in the space frame of the robot is first calculated. This is handled by the `CalculateHomeAndScrewAxes` function at line 10.

The desired end-effector configuration is then set using the detected position of the block with a z-offset for the gripper to touch the top of the block. The orientation is fixed to be parallel to the ground, such that the vacuum gripper is poiting to the negative z direction in the space frame.

The desired end-effector configuration, home configuration, and screw axes in the space frame are  fed into a numerical solver to calculate the inverse kinematics. This numerical solver uses the Newton-Raphson method and an initial guess of zeros for all joint angles. It then returns the desired joint angles of the robot that achieves the given configuration.

The whole functionality of moving the robot to a desired position is handled by the `MoveRobotUsingIK` function defined at line 76.

### Verifying the inverse kinematics calculations by comparing against the forward kinematics

The inverse kinematics solver returns a flag if the numerical method was unable to converge to a solution, which can be used to check if a valid solution was found.

Another sanity check used to verify the calculation is through the forward kinematics. The calculated joint angles are fed into the forward kinematics in order to verify that the position achieved by the solution is the same as the desired position.

### Picking and releasing the object using a vacuum gripper

Once the UR3 reaches the block's position, a vacuum gripper is used to pick up the block. At the end location, the vacuum gripper is turned off to release the block.

The vacuum gripper is controlled by the `VacuumGrip` function at line 69.

## Usage

Usage is still similar as in project update 1. Running the test_control.py controls the simulation.

# Project Update 1 - 10/16/2021

Code for project update 1 is at the [update 1](/update_1) folder.
A [demonstration video](https://youtu.be/9_1yFtfyQoE) shows the current functionalities.

## Functionalities

The sample_scene.ttt CoppeliaSim scene file has a UR3 robot arm, a proximity laser sensor, a white sphere representing a golf ball, and a potted plant. The potted plant can be dragged around to demonstrate that the proximity sensor can detect it.

The Python script named test_control.py is then able to communicate with CoppeliaSim and control the simulation. This script handles starting and stopping the simulation, as well as three main functionalities of interest that is integral to the project.

### Controlling the UR3 and receiving joint feedback

The script is able to control the joint positions of the UR3 robot. For this, the helper function `SetJointAngles` is defined at line 20.

The usage of this function is demonstrated by line 108, which moves the UR3 to the specific joint angles at line 113.

The script is also able to read the current joint positions of the UR3. This is achieved by the helper function `GetJointAngles` defined at line 9.

### Reading distance measurements from proximity sensor

The data from the laser proximity sensor is read using the defined `GetProxSensorDist` helper function at line 28. This function returns whether there is an object detected by the sensor and its detected coordinates.

The usage of this function is demonstrated by the loop in line 121. The pair of sensor readings are then printed out to the console in 1 second intervals, for 15 iterations.

### Spawning a new golf ball

A new sphere is spawned by the script at line 106, which represents the golf ball.

This functionality is necessary as it is not practical to drag and drop the golf ball model while the simulation is playing. The script handles spawning a new golf ball, which allows simulating new golf balls ariving at the pick-up location.

## Usage

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
