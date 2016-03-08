#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import math
import tf
from tf import transformations as transform

rospy.init_node("pose_publisher")
posepub = rospy.Publisher("/in/pose", PoseStamped)

br = tf.TransformBroadcaster()
t = 0.0;
while True:
    t += 0.01;
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now();
    pose.header.frame_id = "/sim_pose"
    q = transform.quaternion_from_euler(t * 0.1, -t * 0.1, t * 0.1)
    pose.pose.orientation.x = q[0];
    pose.pose.orientation.y = q[1];
    pose.pose.orientation.z = q[2];
    pose.pose.orientation.w = q[3];
    pose.pose.position.y = 0.5;
    pose.pose.position.z = math.sin(t) * 0.1 + 0.5;
    pose.pose.position.x = -0.5;
    posepub.publish(pose);
    rospy.sleep(0.01)
    br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), 
                     (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                     pose.header.stamp,
                     "/sim_pose", "/map");
