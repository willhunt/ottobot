#!/bin/sh

# Update
printf "Updating linux...  "
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get autoremove

# Install ROS
# Allow restricted, universe, and multiverse repositories
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
# Setup sources
printf "Adding repositories..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
printf "Installing ROS..."
sudo apt install ros-melodic-ros-base
printf "Setup rosdep..."
sudo rosdep init
rosdep update

# Modify bash.rc to source required files
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/ottobot/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Setup supervisor
sudo apt install supervisor
# Make logging directories
sudo mkdir -p /var/log/ottobot
# Copy setup file
sudo cp -f supervisor/ottobot_ros.conf /etc/supervisor/conf.d/ottobot_ros.conf

printf "Finished."