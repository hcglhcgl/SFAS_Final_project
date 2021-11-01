# SFAS_Final_project
Goal: To find QR codes in an office environment

##Setup code

```bash
roslaunch final_project turtlebot3_world.launch
```
It should open gazebo where you can manually explore and inspect the environment your robot will be navigating. As you will see, there are a number of barriers (see example below) that will spawn in random locations around the room. The barriers are unknown obstacles that any robot has to avoid.
Additionally, 5 QR markers are also spawned randomly in the environment which you will be using according to the project description.


There are a few parameters for launching the environment you can use when you are developing.

1) enable_robot (default true): Will spawn the robot in the environment at launch
2) enable_competition (default true): Will spawn both the random barriers/obstacles and QR-markers
3) layout (default 0): Specifies the layout of the QR markers. You can use this to ensure the QR-markers are always spawned in the same location (e.g. when debugging). Can be either 0 (random), 1, 2, 3, or 4. Note that for the project you hand in, the layout will be 0 and thereby random!
4) gui (default true) & headless (default false): Gazebo is launched with a graphical user interface. You may choose to disable this GUI in favor of computational power and only visualize through other tools such as rviz.

An example that will launch the simulation with a robot, but no QR markers or obstacles/barriers:

```bash
roslaunch final_project turtlebot3_world.launch enable_competition:=false
```

QR markers and obstacles can then be spawned manually with:

```bash
roslaunch final_project spawn_markers.launch
roslaunch final_project spawn_obstacles.launch
```

##Navigation
As we have seen in this course, one of the major advantages of using ROS, is that as long as the interface (i.e. topics) match, we can reuse previous tools very easily! Use the same navigation package you have used previously in the course to navigate our new robot.
With the navigation running, open rviz in another terminal:

```bash
rviz
```

In RViz add the topics that are seen in the screenshot above.

Now you should be able to navigate the robot by pressing the "2D Nav Goal" button in RViz and then pressing somewhere on the map.


##QR Codes
Obviously you'll need to be able to read the QR markers used for the project. First install the following:

```bash
sudo apt install ros-melodic-visp-auto-tracker*
```

Run the command:
```bash
roslaunch final_project qr_visp.launch
```

A new window should now open showing the camera output from the robot. When the QR code is close enough you should see an image like this:

The readings are being published to the topic /visp_auto_tracker/object_position and /visp_auto_tracker/code_message, so you can check if the code is reading by running the following command in a new terminal window:

```bash
rostopic echo /visp_auto_tracker/object_position
rostopic echo /visp_auto_tracker/code_message
```