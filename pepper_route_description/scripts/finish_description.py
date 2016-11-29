#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pnp_msgs.msg import ActionResult
from pepper_route_description.msg import RouteDescriptionAction, RouteDescriptionResult
from pymongo import MongoClient
from std_msgs.msg import String
import tf
from geometry_msgs.msg import PoseStamped
from nao_interaction_msgs.srv import TrackerPointAt, TrackerPointAtRequest
from nao_interaction_msgs.srv import TrackerLookAt, TrackerLookAtRequest
from nao_interaction_msgs.srv import SetBreathEnabled, SetBreathEnabledRequest
import threading


class FinishDescription(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = PNPSimplePluginServer(
            name,
            RouteDescriptionAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        res = RouteDescriptionResult()
        res.result.append(ActionResult(cond="described_route__%s__%s" % (goal.shop_id,goal.waypoint), truth_value=False))
        res.result.append(ActionResult(cond="finished_description__%s__%s" % (goal.shop_id,goal.waypoint), truth_value=True))
        self._as.set_succeeded(res)
        
    def transform_pose(self, target_frame, pose):
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            try:
                self.listener.waitForTransform(target_frame, pose.header.frame_id, rospy.Time.now(), rospy.Duration(1.))
                t = self.listener.getLatestCommonTime(target_frame, pose.header.frame_id)
                pose.header.stamp = t
                return self.listener.transformPose(target_frame, pose)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logdebug(ex)
            
    def set_breathing(self, flag):
        self.__call_service(
            "/naoqi_driver/motion/set_breath_enabled", 
            SetBreathEnabled, 
            SetBreathEnabledRequest(SetBreathEnabledRequest.ARMS, flag)
        )
            
    def __call_service(self, srv_name, srv_type, req):
        while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    srv_name,
                    srv_type
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
                rospy.sleep(1.)
            else:
                return s(req)

if __name__ == "__main__":
    rospy.init_node("finish_description")
    FinishDescription(rospy.get_name())
    rospy.spin()

