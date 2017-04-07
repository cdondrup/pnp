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
from nao_interaction_msgs.srv import Say, SayRequest
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist
from pepper_move_base.msg import TrackPersonAction, TrackPersonGoal
from nao_interaction_msgs.srv import GoToPosture, GoToPostureRequest
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
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
    BASE_LINK = "base_link"
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._as = PNPSimplePluginServer(
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
        rospy.loginfo("Creating move_base client")
        self.move_client = SimpleActionClient("/move_base", MoveBaseAction)
        self.move_client.wait_for_server()
        rospy.loginfo("Move_base client connected")
        rospy.loginfo("Creating tracker client")
        self.start_client = SimpleActionClient("/start_tracking_person", TrackPersonAction)
        self.start_client.wait_for_server()
        self.stop_client = SimpleActionClient("/stop_tracking_person", TrackPersonAction)
        self.stop_client.wait_for_server()
        rospy.loginfo("Tracker client connected")        
        self.db_name = rospy.get_param("~db_name", "semantic_map")
        self.collection_name = rospy.get_param("~collection_name", "idea_park")
        self.semantic_map_name = rospy.get_param("~semantic_map_name")
        self._as.start()
        self.db = client[self.db_name]
        self.tts = rospy.Publisher("/speech", String, queue_size=1)
        self.joints = rospy.Publisher("/joint_angles", JointAnglesWithSpeed, queue_size=10)
        rospy.loginfo("... done")
        
    def stand(self):
        print "STANDING"
        self.__call_service(
            "/naoqi_driver/robot_posture/go_to_posture", 
            GoToPosture, 
            GoToPostureRequest(GoToPostureRequest.STAND_INIT, 0.5)
        )
        j = JointAnglesWithSpeed()
        j.joint_names = ['HeadYaw', 'HeadPitch']
        j.joint_angles = [0.,-.5]
        j.speed = .05
        self.joints.publish(j)
        
    def __call_service(self, srv_name, srv_type, req):
         while not rospy.is_shutdown():
            try:
                s = rospy.ServiceProxy(
                    srv_name,
                    srv_type
                )
                s.wait_for_service(timeout=1.)
            except rospy.ROSException, rospy.ServiceException:
                rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
                rospy.sleep(1.)
            else:
                return s(req)

    def execute_cb(self, goal):
        self.stop_client.send_goal(TrackPersonGoal())
        shop_id = goal.shop_id.split('_')[1]
        result = self.db[self.collection_name].find_one(
            {
                "shop_id": shop_id, 
                "semantic_map_name": self.semantic_map_name
            }
        )
        
        rospy.loginfo("Waiting for current pose from odometry.")
        o = rospy.wait_for_message("/naoqi_driver_node/odom", Odometry)        
        current_pose = PoseStamped()
        current_pose.header = o.header
        current_pose.pose = o.pose.pose
        rospy.loginfo("Received pose.")
        
        loc = PoseStamped()
        loc.header.frame_id = "semantic_map"
        loc.pose.position.x = float(result["loc_x"])
        loc.pose.position.y = float(result["loc_y"])

        target = self.transform_pose(DescribeRoute.BASE_LINK, loc)
        t = TrackerPointAtRequest()
        t.effector = t.EFFECTOR_RARM
        t.frame = t.FRAME_TORSO
        t.max_speed_fraction = 0.5
        t.target.x = target.pose.position.x
        t.target.y = target.pose.position.y
        t.target.z = 1.0
        s1 = ServiceThread("/naoqi_driver/tracker/point_at", TrackerPointAt, t)
        self.set_breathing(False)
        s1.start()
        s1.join(timeout=1.0)

        target = self.transform_pose(DescribeRoute.BASE_LINK, loc)
        t = TrackerLookAtRequest()
        t.use_whole_body = False
        t.frame = t.FRAME_TORSO
        t.max_speed_fraction = 0.6
        t.target.x = target.pose.position.x
        t.target.y = target.pose.position.y
        t.target.z = 1.0
        s2 = ServiceThread("/naoqi_driver/tracker/look_at", TrackerLookAt, t)
        s2.start()
        s1.join()
        s2.join()
        self.pub.publish(Twist())

        current_pose = self.transform_pose(DescribeRoute.BASE_LINK, current_pose)
        current_pose.pose.position.x = 0.
        current_pose.pose.position.y = 0.
        current_pose.pose.position.z = 0.
        
        self.pub.publish(Twist())
        self.__call_service("/naoqi_driver/tts/say", Say, SayRequest(result["directions"]%result["shopName"]))
        self.set_breathing(True)
        g = MoveBaseGoal()
        g.target_pose = current_pose
        self.move_client.send_goal_and_wait(g)
        self.stand()
        self.start_client.send_goal(TrackPersonGoal())
        res = RouteDescriptionResult()
        res.result.append(ActionResult(cond="described_route__%s__%s" % (goal.shop_id,goal.waypoint), truth_value=True))
        res.result.append(ActionResult(cond="finished_description__%s__%s" % (goal.shop_id,goal.waypoint), truth_value=False))
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

if __name__ == "__main__":
    rospy.init_node("describe_route")
    DescribeRoute(rospy.get_name())
    rospy.spin()

