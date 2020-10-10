---
images_firewall: [
    {src: "detail_firewall_01.png", caption: "Firewall rule for robot (192.168.0.21) on PC"},
]
---

# ROS Over Local Network
The official docs are here for reference:
* [Setting up network](http://wiki.ros.org/ROS/NetworkSetup)
* [Running on multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
Useful useful info:
* [Networking for ROS users](https://roscon.ros.org/2013/wp-content/uploads/2013/06/Networking-for-ROS-Users.pdf)
* [Troubleshooting ROS Networking Issues](https://www.youtube.com/watch?v=YMG6DWEqv5g).

## Name Hosts
The easiest way to reference other machines by name is to add them to the `etc/hosts` file:
```bash
$ sudo nano /etc/hosts
```
After the `localhost` entry add the IP address and name (tab separated) for each ROS machine, including itself, to the file. For example here the robot, `ottobot` has IP address `192.168.0.21` and the computer used for observing, `workstation` has address `192.168.0.10`. The host names cannot be abitrary and must be the same as the machines name, seen by using the `hostname` command.
```
127.0.0.1       localhost
192.168.0.10    workstation
192.168.0.21    ottobot
```

If the machines don't have static IP addresses then this won't be effective long term so best to make them static.

**Alternatively** an environment variable can be set (ideally in .bashrc) to let ROS know each machines IP. Either `ROS_IP` or `ROS_HOSTNAME` can be set. For example from the `workstation` be could use:

```bash
workstation:$ export ROS_IP=192.168.0.10
workstation:$ echo "export ROS_IP=192.168.0.10" >> ~/.bashrc
workstation:$ ssh otto@192.168.0.21
    ottobot:$ export ROS_IP=192.168.0.21
    ottobot:$ echo "export ROS_IP=192.168.0.21" >> ~/.bashrc
```
NB: If the names are setup properly in `/etc/hosts` then this is unnecessary. A good explanation is found here: [Troubleshooting ROS Networking Issues](https://www.youtube.com/watch?v=YMG6DWEqv5g).

## Single Master
Roscore only needs (and can) run on one machine. In this case it will be run on the robot, `ottobot` and we will check connection on the PC, `workstation`. In order for the other machines to find the master the environment variable `ROS_MASTER_URI` must be set. 

From `workstation`, connect to `ottobot`, start the roscore, set `ROS_MASTER_URI` ans `ROS_IP`:
```bash
$ ssh otto@ottobot
$ roscore # Not required if already running using supervisor or other method
$ export ROS_MASTER_URI=http://ottobot:11311
```
Now in another terminal (or `exit` ssh) on `worstation`:
```bash
$ export ROS_MASTER_URI=http://ottobot:11311
$ rostopic list
/rosout
/rosout_agg
```
Rostopic should list at least `/rosout` so you know it's found the ROS master on `ottobot`. You can also run `roswtf` as an alternative check which will tell you if a ROS master is found.

It is easiest to set the environment variable in the `.bashrc` file to avoid setting it each time (on the robot anyway, maybe not on your PC where you might want to run `roscore` for other applications).
```bash
$ echo "export ROS_MASTER_URI=http://ottobot:11311" >> ~/.bashrc
```

## Firewall
With names being resolved correctly and each computer finding the ROS master there can still be problems connecting due to firewalls. For me this resulted in not being able to receive ROS messages on my robot from my PC but the PC receiving messages from the robot (and ping working just fine). Using the [netcat test](http://wiki.ros.org/ROS/NetworkSetup) I could chat between both machines if I listened on the robot:

```bash 
ottobot:$ nc -l 38151
```
```bash
workstation:$ nc ottobot 38151
```

But not the other way around:
```bash 
workstation:$ nc -l 38151
```
```bash
ottobot:$ nc workstation 38151
```

The suggested solution is to use a VPN with a guide [here](https://answers.ros.org/question/11045/how-to-set-up-vpn-between-ros-machines/). A quicker way is to allow connections through the firewall. On the `worstation` I allowed incoming connections through the linux mint GUI for `ottobot`'s IP.

<DocsImageLayout :images="$frontmatter.images_firewall" size="lg" srcBase="/ottobot/assets/detail/"></DocsImageLayout>
