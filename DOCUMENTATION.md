## 

> **⚠️ WARNING: Always source ROS 2 and workspace setup**
> 
> - Before proceeding with any setup or launch commands, ensure you have sourced the ROS 2 environment and the workspace:
> 
>   ```bash
>   source /opt/ros/humble/setup.bash  # or setup.zsh if using zsh
>   cd ~/Workspace/block_rearrangement && source install/setup.bash  #   or setup.zsh if using zsh
>   ```
> 
> - Sourcing these setup files configures your shell environment with ROS 2 CLI tools, Package paths and dependencies, and environment variables required for ROS 2 nodes and services. 
>   - Without sourcing, ROS 2 commands and packages from this workspace will not be found.
---
### Setting up the Robot
- Ensure that the UR10 is powered on and is in External Control mode
    - The IP address of each computer connected to the ELPIS network is configured accordingly in each of the external control config file, as shown in the Teach Pendant
    - For Athena PC, the config file is `ext_ctrl_two.urp`

- Check if the Tool Center Point (TCP) is adjusted accordingly as the values may have changed depending on experiments in the lab.
    - For the pick-and-place with the Robotiq gripper, choose the TCP_1 configuration (~153 mm).

- The external control is established by the UR ROS2 driver packages. To bring the necessary nodes online, run the following command:

  ```bash
  ros2 launch ur_elpis_control start_robot.launch.py ur_type:=ur10 robot_ip:=<ip_address>
  ```
    - For Athena PC, the ip_address input is 192.168.0.100, which is the ip address of the UR10 arm on our network
    - If error messages persist, terminate the program and run it again after few seconds. 
      - This behavior is possibly due to improper resource clean-up, re-running should fix it most of the time.
    
- Click the Play Button on the External Control Program Panel to switch over the control to the programs in the PC.

- Finally, to enable the services for motion planning of the UR10 arm through MoveIt's interface, run the following command:
  
  ```bash
  ros2 launch ur_elpis_control complete
  ```
    - This command also opens up the rviz window that provides additional control for the user to plan and execute motion plans as per user request along with some static obstacles in the environment such as table and safety planes.
    - Refer to `ur_elpis_control` package for details on adding/removing the safety planes and static obstacles
---
### Launching the services
- For block rearrangement, the following services are essential for execution
    - Instance-Segmentation to detect the blocks in scene
    - Grounding predicates based on the detections
    - Task Planning to obtain grounded actions
    - MoveIt to generate valid trajectories for the actions
- These services are the backbone for closed-loop or open-loop execution of block arrangement tasks
    - If an existing package or set of packages is/are to be replaced in the future with better ones, make sure that all the backbone services are provided
    - For example, only remove instance-segmentation, predicate grounding, and task planning packages if you are to replace them with an all-in-one package that can provide task plans based on visual inputs 
- Currently, the services in `segmentation`, `grounding`, `task_planner`, and `grasping_moveit` are spun as nodes by the launch file in the `sequencer` package. To bring them online, run:

  ```bash
  ros2 launch sequencer pick_system.launch.py
  ```
- This launches the services of the following servers
    - `segmentation_server`
    - `project_to_3d` 
    - `move_to_home`
    - 
    - 
    -

### Moving Robot to Home Position
- It is imperative that the robot is moved to its home position, as the robot gets a complete view of the region of interest (specific area on the table where the rearrangement is supposed to happen).
- Assuming that the essential services are launched as given in the previous section, run the `move_to_home` client node to use MoveIt planning to execute motion to reach the home configuration
  ```bash
  ros2 launch grasping_moveit move_to_home
  ```

### Spawning Detected Objects
- As of this version, all the planning and task executor is completely dependent on the state of the objects in the MoveIt's Planning Scene. 
    - Hence, for each sensing step executed, we ensure that the planning scene is updated as well.
    - To initially populate the planning scene with blocks (if their poses are known), we need to ensure the spawn service is online.
      ```bash
      ros2 launch spawn_objects spawn_objects_server
      ```
    - If you need to reset the scene, run the following command in another terminal (assuming the service server is spun by the previous command)
      ```bash
      ros2 run spawn_objects clear_objects
      ```
- Once the robot is moved to the home position, the segmentation service needs to be requested for, to detect the blocks in the scene and obtain their poses. This can be done with the following command.
  ```bash
  ros2 run segmentation trigger_segmentation
  ```
    - Everytime this command is run, the `/detections` topic is populated with the latest detections. Continually running the service would block some services that may be needed in parallel, hence the choice for trigger-style service.

    - The `spawn_objects` server continually looks for updates in the `/detections` topic, so once it is updated, the objects in the planning scene are updated as well.
    - Currently, object registry for tracking objects across the scene is not done yet (its implementation can help reground predicates after each execution to correctly populate the scene after mishaps in execution)

### Task Planning and Execution
- The `task_planner` package is responsible for generating task plans for the grounded predicates.
    - The service server calls upon the service server in `grounding` to obtain grounded predicates in each timestep
    - The output of this service is a single grounded action, hence this service is called at each timestep after execution of one action.
    - Currently, we are not closing the loop with another segmentation (need to implement this later)

    - Also, only PDDL style goal specification is supported currently, as we are using PDDL solver to generate task plans.
- The `action_executor` service server in `task_planner` spins different executor threads for each type of action
    - If new actions are to be introduced, follow how the other action classes are written
- Each action class ultimately makes a service call to the `Moveit Task Constructor` based server that provides motion plans to sequentially execute the sub-steps involved in a pick task and a place task.
- Run the following command to execute the rearrangement execution phase
  ```bash
  ros2 run grasping_moveit action_executor "goal"
  ``` 
    - Example of how goal string needs to be passed as argument : " and ((ontable cube_1) (on cube_2 cube_0))"
    - If execution fails due to non-feasible motion plan (invalid ik solution for goal pose), try running the `action_executor` node with the same goal again.
    - This can be the case frequently when `put-down` action is grounded, as it samples 100 random goal poses to place object on table, out of which there can be none that are collision-free

### Common Pitfalls

### Closing the Programs
- Please do not leave the services running continually as others may want to use the robot or the camera for their experiments
- Please ensure that you stop the services runnning in each terminal by manual keyboard interrupts
- Stop the External Control program on the teach pendant of the UR10 arm and switch off the robot if you are done with the demonstration or testing.
- Switch off the robot :)

