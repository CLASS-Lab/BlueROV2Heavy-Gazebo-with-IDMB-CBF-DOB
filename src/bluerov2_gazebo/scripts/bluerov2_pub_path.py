#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


from block_warning import suppress_TF_REPEATED_DATA

path = Path()

def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

rospy.init_node('path_pub_node')
# useless
# suppress_TF_REPEATED_DATA()
# rospy.INFO("start pub path")
odom_sub = rospy.Subscriber('/bluerov2/pose_gt', Odometry, odom_cb)
path_pub = rospy.Publisher('/bluerov2/path', Path, queue_size=20)

if __name__ == '__main__':
    rospy.spin()