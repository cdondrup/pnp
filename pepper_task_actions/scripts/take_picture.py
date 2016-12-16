#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer, SimpleActionClient
from pepper_task_actions.msg import EmptyAction
from nao_interaction_msgs.srv import BehaviorManagerControl, BehaviorManagerControlRequest
from nao_interaction_msgs.srv import BehaviorManagerInfo, BehaviorManagerInfoRequest
from pepper_move_base.msg import TrackPersonAction, TrackPersonGoal
from nao_interaction_msgs.srv import SetBreathEnabled, SetBreathEnabledRequest



class TakePicture(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(name, EmptyAction, self.execute_cb, auto_start=False)
        self.app_name = rospy.get_param("~app_name", "pose-photo/pose-photo-interactive")
        rospy.loginfo("Creating tracker client")
        self.start_client = SimpleActionClient("/start_tracking_person", TrackPersonAction)
        self.start_client.wait_for_server()
        self.stop_client = SimpleActionClient("/stop_tracking_person", TrackPersonAction)
        self.stop_client.wait_for_server()
        rospy.loginfo("Tracker client connected") 
        self._as.start()
        rospy.loginfo("... done")
        
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
        self.__call_service(
            "/naoqi_driver/behaviour_manager/start_behaviour", 
            BehaviorManagerControl, 
            BehaviorManagerControlRequest(name=self.app_name)
        )
        rospy.sleep(1.)
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._as.is_preempt_requested():
            if self.app_name not in self.__call_service(
                "/naoqi_driver/behaviour_manager/get_running_behaviors", 
                BehaviorManagerInfo, 
                BehaviorManagerInfoRequest()
            ).behaviors:
                break
            r.sleep()
        self.set_breathing(False, SetBreathEnabledRequest.BODY)
        self.start_client.send_goal(TrackPersonGoal())
        self.set_breathing(True)
        self._as.set_succeeded()
        
    def set_breathing(self, flag, actuator=SetBreathEnabledRequest.ARMS):
        self.__call_service(
            "/naoqi_driver/motion/set_breath_enabled", 
            SetBreathEnabled, 
            SetBreathEnabledRequest(actuator, flag)
        )

if __name__ == "__main__":
    rospy.init_node("take_picture")
    TakePicture(rospy.get_name())
    rospy.spin()

