# Debugging ROS

## Build 
To get maximum information from the debugger nodes must be built in debug mode:
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Debug
```
This can be reverted by using:
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=None
```

## Running with GDB
To run a node with GDB the prefix `gdb -ex run` must be used. This can be implemented in the launch file using a debug parameter.

```xml
<arg name="debug" default="true" />
<node name="my_node" pkg="my_package" type="my_node" respawn="false" output="screen"
    launch-prefix="$(eval 'gdb -ex run' if debug else '')"> 
</node>
```

Other prefixes with different options are listed [here](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB) on the ROS wiki.
