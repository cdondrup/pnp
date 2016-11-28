#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer
from pepper_route_description.msg import RouteDescriptionAction, RouteDescriptionResult
from pymongo import MongoClient
from std_msgs.msg import String
import tf
from geometry_msgs.msg import PoseStamped
from nao_interaction_msgs.srv import TrackerPointAt, TrackerPointAtRequest
from nao_interaction_msgs.srv import TrackerLookAt, TrackerLookAtRequest
from nao_interaction_msgs.srv import SetBreathEnabled, SetBreathEnabledRequest
import threading


class ServiceThread(threading.Thread):
    def __init__(self, srv_name, srv_type, srv_req):
        super(ServiceThread, self).__init__()
        self.srv_name = srv_name
        self.srv_type = srv_type
        self.srv_req = srv_req
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    self.srv_name,
                    self.srv_type
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % self.srv_name)
                rospy.sleep(1.)
            else:
                s(self.srv_req)
                break


class DescribeRoute(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(
            name,
            RouteDescriptionAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.listener = tf.TransformListener()
        client = MongoClient(
            rospy.get_param("~db_host", "localhost"),
            int(rospy.get_param("~db_port", 62345))
        )
        self.db_name = rospy.get_param("~db_name", "semantic_map")
        self.collection_name = rospy.get_param("~collection_name", "idea_park")
        self.semantic_map_name = rospy.get_param("~semantic_map_name")
        self._as.start()
        self.db = client[self.db_name]
        self.tts = rospy.Publisher("/speech", String, queue_size=1)
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        result = self.db[self.collection_name].find_one(
            {
                "shop_id": goal.shop_id, 
                "semantic_map_name": self.semantic_map_name
            }
        )
        
        loc = PoseStamped()
        loc.header.frame_id = "semantic_map"
        loc.pose.position.x = float(result["loc_x"])
        loc.pose.position.y = float(result["loc_y"])

        target = self.transform_pose("base_link", loc)
        t = TrackerPointAtRequest()
        t.effector = t.EFFECTOR_RARM
        t.frame = t.FRAME_TORSO
        t.max_speed_fraction = 0.1
        t.target.x = target.pose.position.x
        t.target.y = target.pose.position.y
        s1 = ServiceThread("/naoqi_driver/tracker/point_at", TrackerPointAt, t)
        self.set_breathing(False)
        s1.start()
        s1.join(timeout=1.0)
        
        target = self.transform_pose("base_link", loc)
        t = TrackerLookAtRequest()
        t.use_whole_body = False
        t.frame = t.FRAME_TORSO
        t.max_speed_fraction = 0.6
        t.target.x = target.pose.position.x
        t.target.y = target.pose.position.y
        s2 = ServiceThread("/naoqi_driver/tracker/look_at", TrackerLookAt, t)
        s2.start()
        s1.join()
        s2.join()

        self.tts.publish(result["directions"])
        rospy.sleep(3.)
        self.set_breathing(True)
        self._as.set_succeeded(RouteDescriptionResult())
        
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
    rospy.init_node("describe_route")
    DescribeRoute(rospy.get_name())
    rospy.spin()

