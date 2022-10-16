# esce373_f22_group8_stdr_driver_node

This package is partially dependent on the stdr_simulator ROS package:

https://github.com/cwru-eecs-275/stdr_simulator


**How to run this ROS package:**

    roslaunch stdr_driver_node launchfilename.launch

If you run the first launch file (robot_launch.launch), you will then also need to run:

    roslaunch stdr_launchers server_with_map_and_gui.launch.

Next, create a robot in the stdr simulator gui, add a laser, and add it to the map.

To control the robot, run:

    rosrun rqt_gui rqt_gui

 - in this gui, select Plugins -> Robot Tools -> Robot Steering
 - make sure to have "/robot0/des_vel" in the textbox at the top


**How to setup control for multiple robots**
1. Run the initial roslaunch with the sim:

      roslaunch stdr_driver_node robot_launch_w_sim.launch

2. This will start the stdr_driver_node for robot0 and open the STDR simulator rqt_gui
3. Next, add robot to the STDR rqt_gui:
    - in the stdr simulator gui, click "create robot" in the toolbar
    - then click the green plus next to laser.
    - next, click "add robot to map" at the bottom of the window
    - then click on the map to place the robot
4. Repeat step 3 to place additional robots in the map (robot1, robot2, etc...)
5. Run the roslaunch to create the stdr_driver_node for the additional robots
        roslaunch stdr_driver_node robot_launch.launch robot_ns:="robot1"
6. Repeat steps 3-5 for additional robots, incrementing the robot_ns argument when launching the stdr_driver_node

**How it works:**

- The stdr_driver_node will read the desired velocities from the rqt_gui and use them to drive the robot.
- When the robot approaches a wall, a warning will be shown in the terminal
When the robot gets too close to a wall, the robot will set its forward velocity to 0, overriding the rqt_gui

**Messages seen on the Terminal**
1. [FATAL] Add a robot with lidar!!
  - in the stdr simulator gui, click "create robot" in the toolbar
  - then click the green plus next to laser.
  - next, click "add robot to map" at the bottom of the window
  - then click on the map to place the robot
2. [INFO] You're Good...
  - the command velocity is equal to the desired from the rqt_gui
  - the robot is not too close to a wall
3. [WARN] slow down!! wall ahead
  - the robot is getting close to a wall
4. [ERROR] Too close to wall, stop
  - the robot is too close to a wall
  - the desired forward velocity is overridden and set to zero

**Launch File Options**
1. robot_launch.launch
    - this launch file creates a node with the following properties:
    - name="stdr_driver_node"
    - type="stdr_driver_node"
    - pkg="stdr_driver_node"
    - output="screen"
    - ns="$(arg robot_ns)"      (ns = namespace)
    - **Arguments for launch file**
      - robot_ns
        - default = "robot0"
        - for additional robots, change to robot1, robot2, etc...



2. robot_launch_w_sim.launch
    - includes 2 other launch files
      1. robot_launch.launch with properties shown above
      2. server_with_map_and_gui.launch
        - this launch file is in the stdr_simulator ROS package linked above
        - launches the STDR simulator gui
