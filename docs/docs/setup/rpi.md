# Raspberry Pi Setup
Useful resources:
* Install Ubuntu Mate: [The Robotics Back-End](https://roboticsbackend.com/install-ubuntu-mate-18-04-on-raspberry-pi-3-b/)
* Install ROS: [The Robotics Back-End](https://roboticsbackend.com/install-ros-on-raspberry-pi-3/)

## Install Ubuntu Server 18
Install Ubuntu Server: [Ubuntu Official Guide](https://ubuntu.com/download/raspberry-pi)

## Connecting
Connect to raspberry pi via ssh:
```bash
$ ssh <username>@<ip address>
```
In my case:
```bash
$ ssh ubuntu@192.168.0.21
```
Add a new user if desired, in this cas `otto`:
```bash
$ sudo adduser otto
$ sudo usermod -a -G sudo otto
```
Follow prompts and then reconnect with new user:
$ exit
$ ssh otto@192.168.0.21
```

## Additional setup with script
Once Ubuntu is installed, download the GitHub repository
```bash
$ git clone https://github.com/willhunt/ottobot.git
$ cd ottobot/setup
$ chmod +x setup_raspi.sh
$ ./setup_raspi.sh
```


## Catkin Build
Building large nodes with all cores can cause issues so specifying 2 can be faster and more reliable:
```bash
$ catkin_make -j2
```