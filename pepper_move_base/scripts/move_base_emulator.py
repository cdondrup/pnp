#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer
from move_base_msgs.msg import MoveBaseAction
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np


class MoveBase(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.odom_topic = rospy.get_param("~odom_topic", "/naoqi_driver_node/odom")
        self.listener = tf.TransformListener()
        self.robot_pose = None
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self._as = SimpleActionServer(name, MoveBaseAction, self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
        target_pose = goal.target_pose
        if target_pose.header.frame_id != self.target_frame:
            bl_pose = self.transform_pose(self.target_frame, target_pose)
        else:
            bl_pose = target_pose
        if bl_pose != None:
            self.pub.publish(bl_pose)
            odom_pose = self.transform_pose("odom", bl_pose)
            r = rospy.Rate(10)
            start = rospy.Time.now().to_sec()
            try:
                while not rospy.is_shutdown() and not self._as.is_preempt_requested():
                    if self.get_euclidean_distance(odom_pose.pose, self.robot_pose.pose.pose) < 0.15:
                        break
                    elif rospy.Time.now().to_sec() > start + rospy.get_param("~timeout", 20.):
                        raise rospy.ROSException
                        break
                    r.sleep()
            except rospy.ROSException:
                rospy.logwarn("Navigation timed out")
                self._as.set_aborted()
            else:
                self._as.set_succeeded()
        else:
            print "error"
            self._as.set_aborted()
        
        
    def odom_cb(self, msg):
        self.robot_pose = msg
            
    def transform_pose(self, target_frame, pose):
        try:
            t = self.listener.getLatestCommonTime(target_frame, pose.header.frame_id)
            pose.header.stamp = t
            return self.listener.transformPose(target_frame, pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
            rospy.logwarn(ex)
            return None
            
    def get_euclidean_distance(self, pose1, pose2):
        return np.sqrt(np.square(pose2.position.x-pose1.position.x)+np.square(pose2.position.y-pose1.position.y))


if __name__ == "__main__":
    rospy.init_node("move_base")
    MoveBase(rospy.get_name())
    rospy.spin()

