# ROS Tools
Always:
```bash
$ cd catkin_ws
$ source devel/setup.bash
```

## rostopic
[wiki](http://wiki.ros.org/rostopic)
Useful commands
```bash
$ rostopic list               // list topics
$ rostopic info <topic-name>  // pub/subs
$ rostopic echo <topic-name>  // See output
```

## RQT
```bash
$ rqt
```

## RQT Graph
```bash
$ rosrun rqt_graph rqt_graph
```

## Nav Goal
Useful command to cancel navigation goal
```bash
$ rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}
```