Install turtlebot

sudo apt install ros-eloquent-cartographer
sudo apt install ros-eloquent-cartographer-ros
sudo apt install ros-eloquent-navigation2
sudo apt install ros-eloquent-nav2-bringup
sudo apt install python3-vcstool

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos
colcon build --symlink-install

echo 'source ~/turtlebot3_ws/install/setup.zsh' >> ~/.zshrc

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py

echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.zshrc

ros2 run nav2_util lifecycle_bringup map_server
