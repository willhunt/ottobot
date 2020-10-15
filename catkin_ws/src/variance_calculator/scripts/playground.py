from nav_msgs.msg import Odometry
from operator import attrgetter

odom_msg = Odometry()
selectors = "pose/pose/position".split("/")
variable = attrgetter(".".join(selectors))(odom_msg)
print(variable)
