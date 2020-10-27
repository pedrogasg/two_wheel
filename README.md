
Tested in Ubuntu Focal only

## Prepare the computer

### Install git and others tools

> sudo apt install -y git curl vim

### Add Oh my sh for confort

> sudo apt install -y zsh

> sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

### Install cuda 10.1 and cudnn

> sudo apt install -y nvidia-cuda-toolkit

You need to download [cuDNN for Linux](https://developer.nvidia.com/rdp/cudnn-download)(you need to be logged currently the version is 8.0.4.30)

> tar -xvzf cudnn-10.1-linux-x64-v8.0.4.30.tgz

> sudo cp cuda/include/cudnn.h /usr/lib/cuda/include/

> sudo cp cuda/lib64/libcudnn* /usr/lib/cuda/lib64/

> sudo chmod a+r /usr/lib/cuda/include/cudnn*.h /usr/lib/cuda/lib64/libcudnn*

## Install ros 2


#### Update locales

> locale  # check for UTF-8
>
> sudo apt update && sudo apt install locales
>
> sudo locale-gen en_US en_US.UTF-8
>
> sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
>
> export LANG=en_US.UTF-8
>
> locale  # verify settings

### Setup sources

> sudo apt update && sudo apt install -y gnupg2 lsb-release
>
> curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
>
> sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

### Install packages

> sudo apt update
>
> sudo apt install -y ros-foxy-desktop
>
> echo 'source /opt/ros/foxy/setup.zsh' >> ~/.zshrc
>
> sudo apt install -y python3-pip
> 
> pip3 install -U argcomplete
>
> sudo apt install -y python3-colcon-common-extensions

## Install and run two_wheel

> echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.zshrc
> 
> sudo apt install ros-foxy-joint-state-publisher
>
> sudo apt install ros-foxy-joint-state-publisher-gui
>
> sudo apt install -y ros-foxy-gazebo-ros-pkgs
>
> sudo apt install ros-foxy-xacro
>
> sudo apt install -y ros-foxy-navigation2
>
> sudo apt install -y ros-foxy-nav2-bringup

> colcon build

> . ./install/setup.zsh

> ros2 launch two_wheel_urdf rviz.launch.py

> 