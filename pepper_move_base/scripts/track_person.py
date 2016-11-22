#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_move_base.msg import TrackPersonAction, TrackPersonResult
from pnp_msgs.msg import ActionResult
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nao_interaction_msgs.msg import PersonDetectedArray
from nao_interaction_msgs.srv import SetTrackerTarget, SetTrackerTargetRequest
from nao_interaction_msgs.srv import SetTrackerMode, SetTrackerModeRequest
from nao_interaction_msgs.srv import StartTracker, StartTrackerRequest
from std_srvs.srv import Empty, EmptyRequest
import math
import numpy as np


class PeopleTracking(object):
    sub = None
    
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    def start(self):
        PeopleTracking.sub = rospy.Subscriber("/naoqi_driver_node/people_detected", PersonDetectedArray, self.callback)
        
    def stop(self):
        PeopleTracking.sub.unregister()
        
    def callback(self, msg):
        for p in msg.person_array:
            if p.id == int(self.id):
                try:
                    t = self.listener.getLatestCommonTime(self.target_frame, msg.header.frame_id)
                    p_pose = PoseStamped(header=msg.header, pose=p.person.position)
                    p_pose.header.stamp = t
                    bl_pose = self.listener.transformPose(self.target_frame, p_pose)
                except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                    rospy.logwarn(ex)
                else:
                    theta = math.atan2(bl_pose.pose.position.y, bl_pose.pose.position.x)
                    t = Twist()                    
                    if np.abs(theta) > 0.1:
                        t.angular.z = theta
                    self.pub.publish(t)
                finally:
                    break
                    
    def _call_service(self, srv_name, srv_type, req):
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


class StartStopPeopleTracking(PeopleTracking):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        super(StartStopPeopleTracking, self).__init__()
        self.listener = tf.TransformListener()
        self.id = 0
        self.target_frame = "base_link"
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=TrackPersonAction,
            execute_cb=self.start_cb if "start" in name else self.stop_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("... done")
        
    def start_cb(self, goal):
        self.id = goal.id.split('_')[1]
        self.start()
        req = SetTrackerTargetRequest()
        req.target = req.PEOPLE
        req.values.append(float(self.id))
        self._call_service("/naoqi_driver/tracker/register_target", SetTrackerTarget, req)
        self._call_service("/naoqi_driver/tracker/set_mode", SetTrackerMode, SetTrackerModeRequest(SetTrackerModeRequest.HEAD))
        self._call_service("/naoqi_driver/tracker/track", StartTracker, StartTrackerRequest(StartTrackerRequest.PEOPLE))
        res = TrackPersonResult()
        res.result.append(ActionResult(cond="tracking__"+goal.id, truth_value=True))
        res.result.append(ActionResult(cond="no_tracking", truth_value=False))
        self._ps.set_succeeded(res)
        
    def stop_cb(self, goal):
        self.stop()
        self._call_service("/naoqi_driver/tracker/stop_tracker", Empty, EmptyRequest())
        self._call_service("/naoqi_driver/tracker/unregister_all_targets", Empty, EmptyRequest())
        self.pub.publish(Twist())
        res = TrackPersonResult()
        res.result.append(ActionResult(cond="tracking__"+goal.id, truth_value=False))
        res.result.append(ActionResult(cond="no_tracking", truth_value=True))
        self._ps.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("track_person")
    start = StartStopPeopleTracking("start_tracking_person")
    stop = StartStopPeopleTracking("stop_tracking_person")
    rospy.spin()

