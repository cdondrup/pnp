#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_move_base.msg import QualitativeMovePepperAction, QualitativeMovePepperResult
from pnp_msgs.msg import ActionResult
from nao_interaction_msgs.msg import PersonDetectedArray
import tf
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml
import math
from copy import deepcopy


class QualitativeMove(PNPSimplePluginServer):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.listener = tf.TransformListener()
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        with open(rospy.get_param("~config_file"),'r') as f:        
            self.distance = yaml.load(f)["distances"]
        self.move_client = SimpleActionClient("move_base", MoveBaseAction)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=QualitativeMovePepperAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
        try:
            msg = rospy.wait_for_message("/naoqi_driver_node/people_detected", PersonDetectedArray, timeout=5.)
        except rospy.ROSException as e:
            rospy.logwarn(e)
            self._ps.set_aborted()
        else:
            int_id = int(goal.id.split("_")[1])
            for p in msg.person_array:
                if p.id == int_id:
                    try:
                        t = self.listener.getLatestCommonTime(self.target_frame, msg.header.frame_id)
                        p_pose = PoseStamped(header=msg.header, pose=p.person.position)
                        p_pose.header.stamp = t
                        bl_pose = self.listener.transformPose(self.target_frame, p_pose)
                    except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                        rospy.logwarn(ex)
                    else:
                        target_dist = p.person.distance - self.distance[goal.to]
                        d = target_dist/p.person.distance
                        theta = math.atan2(bl_pose.pose.position.y, bl_pose.pose.position.x)
                        target_pose = bl_pose
                        target_pose.pose.position.x *= d
                        target_pose.pose.position.y *= d
                        target_pose.pose.orientation.x = 0.
                        target_pose.pose.orientation.y = 0.
                        target_pose.pose.orientation.z = math.sin(theta/2.)
                        target_pose.pose.orientation.w = math.cos(theta/2.)
                        print target_pose
                        self.move_client.send_goal_and_wait(MoveBaseGoal(target_pose=target_pose))
                    finally:
                        break
            self._ps.set_succeeded()


if __name__ == "__main__":
    rospy.init_node("goto")
    QualitativeMove(rospy.get_name())
    rospy.spin()

