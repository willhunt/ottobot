#!/bin/sh

# Update
printf "Updating linux...  "
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get autoremove

printf "Setting up static IP...  "
sudo cat >> /etc/dhcpcd.conf <<EOL

# Setup static IP
interface wlan0
static ip_address=192.168.0.21/24
static routers=192.168.0.1
static domain_name_servers=192.168.0.1
EOL

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
sudo apt install ros-melodic-ros-base python-rosdep2 gdb -y
printf "Setup rosdep..."
cd ~/ottobot/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Modify bash.rc to source required files
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/ottobot/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Setup supervisor
sudo apt install supervisor -y
# Make logging directories
sudo mkdir -p /var/log/ottobot
# Copy setup file
sudo cp -f supervisor/ottobot_ros.conf /etc/supervisor/conf.d/ottobot_ros.conf

# Setup host names
printf "Setting up host names...  "
sudo cat >> /etc/hosts <<EOL

192.168.0.10    workstation
192.168.0.21    ottobot
EOL

echo "export ROS_MASTER_URI=http://ottobot:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=ottobot" >> ~/.bashrc

# Serial
printf "Setting up serial...  "
sudo usermod -a -G dialout otto

# Revert serial client
printf "Reverting SerialClient.py to kinetic to work with rosserial_arduino...  "
sudo mv kineticSerialClient.py /opt/ros/melodic/lib/python2.7/dist-packages/rosserial_python/kineticSerialClient.py
cd /opt/ros/melodic/lib/python2.7/dist-packages/rosserial_python
sudo mv SerialClient.pyc melodicSerialClient.pyc
sudo mv SerialClient.py melodicSerialClient.py
sudo mv kineticSerialClientc.py SerialClient.py

printf "Finished. Restart not (sudo reboot)."