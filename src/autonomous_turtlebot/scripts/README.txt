############### Important Notes ###################
Note that there are extra files in this folder.
There were used to help test and debug the three major files:

PID_controller.py
RRT_node.py
PathPlanner_node.py

There is a particularly handy file called visualize_path.py, which publishes
a marker to the robot_path topic. This helps visualize the path found by the
RRT algorithm! You will need to add a marker tool in the rviz menu and specify
the topic name inorder to see it!

################ Launch File ####################
I made a launch file to demonstrate the algorithms in action!

You can run this launch file using the following command:

roslaunch autonomous_turtlebot launch.launch

This launch file calls a map_provider.launch file in another package. You 
can choose the yaml file and stage you want to test the algorithms on here. But
make sure to also specify the same stage in the launch.launch file.

IMPORTANT! The map_provider loads the map and publishes it to rviz! However, rviz
may refresh itself and overwrite it! To see the map indefinitely, one can do 
the following:

roscd turtlebot3_slam/launch 

gedit turtlebot3_hector.launch 

once here we can:
change the map_pub_period parameter value to a large number : 99999 works

this will now allow us to display our complete map indefinitely on rviz

The default map is stage_2. 

I have included yaml and pgm files for stages 2,3, and 4. However, this can
be tested with any other pgm and yaml files if provided. Make sure the changes
are specified in the launch files accordingly for any other map or stage.

############### Test Position ##################

The launch file launches a node called test_position.py
One can specify the particular goal coordinate for the turtlebot here by going 
into the file and editing it.

############### Potential Bugs & Closing Notes ##################

The RRT algorithm works very well! However, sometimes, it can reach a dead end
if it chooses a wrong configuration point (especially in stage 4). I unfortunately did not have time
to fix this bug. But it is usually resolved by running the RRT node again 
(or relaunching the nodes together)

Unfortunately, I was not able to finish method 2 for PID control. However, PID
control method 1 works splendily.

 


