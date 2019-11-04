# gwendolen-ros-curiosity
Examples of a Gwendolen agent autonomously controlling a Mars Curiosity Rover.

Curiosity model and Mars world courtesy of [The Construct](https://bitbucket.org/theconstructcore/curiosity_mars_rover/src/master/).

**Tested on Ubuntu 16.04 with ROS Kinetic and Ubuntu 18.04 with ROS Melodic** (it might work on different versions, but there is no confirmation yet).

Requires [MCAPL](https://github.com/mcapl/mcapl) to be installed and tested to run correctly.

Requires [Gwendolen-Rosbridge](https://github.com/autonomy-and-verification-uol/gwendolen-rosbridge) to be installed and tested to run correctly.

## Inspection

To run the Inspection simulation:
1. Copy the `curiosity_mars_rover_description` folder to your `src` folder of your catkin workspace
   * If you don't have a catkin workspace you can follow [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) (adapt for your version of ROS)
   * Make sure you either `source ~/catkin_ws/devel/setup.bash` (assuming `catkin_ws` is the path to your catkin workspace) everytime you wish to launch race world on a new terminal, or add it to your `~/.bashrc`
2. Recompile your catkin workspace by going to `~/catkin_ws/` (assuming `catkin_ws` is the path to your catkin workspace) and running `catkin_make`
   * To test that the simulation is working, run `roslaunch curiosity_mars_rover_description main_real_mars.launch` and press play on the Gazebo window (the curiosity shouldn't do anything, but there should be no errors in the terminal)
3.  Copy the `src/examples/gwendolen/ros/curiosity/inspection` folder from this git to MCAPL root
4. Launch the simulation in ros `roslaunch curiosity_mars_rover_description main_real_mars.launch` but do not press play yet
5. If you installed MCAPL in Eclipse, go to `src/examples/gwendolen/ros/curiosity/inspection`, right-click curiosity.ail, select run as > run configurations, type run-AIL in the search box (should be there if MCAPL was installed correctly), and click on run

**or**

If you prefer running from the command line, edit the file build.xml on the root folder of mcapl to add this line in the <fileset> group of <path>: <include name="lib/3rdparty/java_rosbridge_all.jar"/>, and run ant compile on the root folder of mcapl. Then, from the mcapl root folder, use this command to run the agent `java -cp .:bin:lib/3rdparty/RunJPF.jar:lib/3rdparty/java_rosbridge_all.jar:lib/3rdparty/system-rules-1.16.0.jar:lib/3rdparty/junit-4.10.jar:lib/3rdparty/json-simple-1.1.1.jar:lib/3rdparty/jpl.jar:lib/3rdparty/jpf-classes.jar:lib/3rdparty/jpf-annotations.jar:lib/3rdparty/java-prolog-parser.jar:lib/3rdparty/ev3tools.jar:lib/3rdparty/ev3classes.jar:lib/3rdparty/eis-0.5.0.jar:lib/3rdparty/jpf.jar:lib/3rdparty/antlr-4.7-complete.jar:lib/3rdparty/commons-io-2.4.jar:lib/3rdparty/mapdbWrapper.jar ail.mas.AIL src/examples/gwendolen/ros/curiosity/inspection/curiosity.ail`
  
6. Once ROS and the agent are running, press play on the Gazebo window
  * The curiosity will patrol four different waypoints
