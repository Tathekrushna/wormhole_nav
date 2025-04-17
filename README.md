# amr_simulation_ros

Go to terminal and run:
1. `sudo apt install git -y`
2. `mkdir -p ~/ros2_ws/src`
3. `cd ~/ros2_ws/src`
4. `git clone <SSH_Key_of_github_repo>'`
5. `cd ..`
6. `colcon build --symlin-install`
7. `source install/setup.bash `

**Important:**

Now follow these steps:
1. Open a new terminal and run `gazebo`.
2. Go to the "Insert" tab at the top of left bar.
3. From the drop-down menu of your "GAZEBO_MODEL_PATH" directory, add `Basic Mobile Robot` and `w(world)` to the gazebo environment.
4. After this, you can close Gazebo.

**SLAM:** 

1. Run `ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py slam:=True`.
2. For keyboard control, run `ros2 run toto2_teleop teleop_keyboard` **OR** run `ros2 launch toto2_description joystick.launch.py` to control using PS4 controller.
3. To save your map, run to `cd ros2_ws/src/basic_mobile_robot/maps/` and then to save map run `ros2 run nav2_map_server map_saver_cli -f ~/{name_of_your_map}`

**Navigation:**

**If you are using your own map**, follow these steps before starting the Navigation Stack:

1. Change `static_map_path = os.path.join(pkg_share, 'maps', 'r1.yaml')` to `static_map_path = os.path.join(pkg_share, 'maps', '{name_of_your_map}.yaml')` in **/basic_mobile_robot/launch/basic_mobile_bot_v5.launch.py**.

To launch the navigation stack:

1. Run `ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py`.
2. Click the **2D Pose Estimate button** in the RViz2 menu. (Position of the robot will be near outside wall and below main gate of the house).
3. Click on the map where the robot is located in gazebo and drag the large green arrow toward the direction where the robot is facing.
4. After this, click the **Navigation2 Goal** button in the RViz2 menu.
5. Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.

**MUltimap Navigation with Wormhole:**

1. Change the map of robot `static_map_path = os.path.join(pkg_share, 'maps', 'r1.yaml')` to `static_map_path = os.path.join(pkg_share, 'maps', '{name_of_your_map}.yaml')` in **/basic_mobile_robot/launch/basic_mobile_bot_v5.launch.py**.
2. Change the wormhole position in 'wormholes.db'
3. Change the name of current map, targetmap and goal position in 'wormhole_action_server.cpp'

To launch the Multimap and wormhole navigation ststem:
1. Run `ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py`.
2. then in another terminal run 'ros2 run wormhole_nav wormhole_action_server'

