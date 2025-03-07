
**FULL PIPELINE - Nodes/Commands to run in each terminal:**
* rviz2
* The bot itself 
    * For Sim —> ros2 launch interbotix_xslocobot_moveit xslocobot_moveit.launch.py robot_model:=locobot_wx200 use_lidar:=true hardware_type:=gz_classic
    * For Real Robot --> ???  
* The arm wrapper
    * For sim —> ros2 launch locobot_wrapper arm_control.launch.py
    * For Real Robot —> ros2 launch locobot_wrapper arm_control.launch.py use_sim:=false
* The manipulation node
    * For sim —> ros2 run locobot_autonomy manipulation_node.py
    * For Real Robot --> _same_ (expected, though not tested. Assumes relevant topics wont change)
* The driver node
    * For sim —> ros2 run locobot_autonomy node_driver_with_sim.py
    * For Real Robot -> ros2 run locobot_autonomy node_driver_with_sim.py use_sim:=false **(code not tested)**
* The localization node
    * For sim —> ros2 run locobot_autonomy localization_tp.py
    * For Real Robot -> _same_ (expected, though not tested. Assumes relevant topics won't change)
* The camera node. THIS IS THE MAIN DRIVER
    * For sim —> ros2 run locobot_autonomy camera_with_sim.py
    * For Real Robot -> _same_ (expected, though not tested. Assumes relevant topics won't change. If they do, change use_sim in code to false)

NOTES ON CODE CHANGES
* Several syntactical to interface with ROS2 msg objects correctly
* Testing showed images are indexed with origin at top left, positive x right, positive y down. So image[y,x]!
* Testing showed align_depth does not handle pure rotations (which don't alter image) well. Added guards for this to align_depth function. Also, right now, image and depth don't really need aligning, so not confirmed that this code works.
* Testing showed K matrix missing centerpoints. Altered localization_tp to account for this.
* Controller can be tuned in node_driver
