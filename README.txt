Lancement manuel!

Terminal 1: On lance la simulation

cd ~/ros2_ws/src/gazebo_keyboard_cmdvel/world
gz sim sensor_tutorial.sdff




Terminal 2: On lance le bridge des déplacements (ROS2 vers Gazebo)

source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist




Terminal 3: On lance le bridge des touches clavier (Gazebo vers ROS2)

source ~/ros2_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32



Terminal 4: On exécute le noeud

source ~/ros2_ws/install/setup.bash
ros2 run gazebo_keyboard_cmdvel keyboard_to_cmdvel
##Si on veut exécuter avec les paramètres spéciaux du yaml, ajouter: --ros-args --params-file /home/nico/ros2_ws/install/gazebo_keyboard_cmdvel/share/gazebo_keyboard_cmdvel/config/keyboard_to_cmdvel.yaml



Lancement avec le launch spécifique et le yaml:


Terminal 1:
gz sim sensor_tutorial.sdf

Terminal 2: 
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_keyboard_cmdvel keyboard_gz.launch.py



Lancement avec le launch générique:

#Pour lancer gazebo indépendamment

Terminal 1:
gz sim sensor_tutorial.sdf

Terminal 2:
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_keyboard_cmdvel launch_general.launch.py

#Pour lancer gazebo en même temps

Terminal 1:
source ~/ros2_ws/install/setup.bash
ros2 launch gazebo_keyboard_cmdvel launch_general.launch.py \
launch_gazebo:=True \
world:=/home/nico/ros2_ws/src/gazebo_keyboard_cmdvel/world/sensor_tutorial.sdf

