#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer
from pepper_task_actions.msg import EmptyAction
from nao_interaction_msgs.srv import BehaviorManagerControl, BehaviorManagerControlRequest
from nao_interaction_msgs.srv import BehaviorManagerInfo, BehaviorManagerInfoRequest


class TakePicture(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(name, EmptyAction, self.execute_cb, auto_start=False)
        self.app_name = rospy.get_param("~app_name", "pose-photo/pose-photo-interactive")
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
        self._as.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("take_picture")
    TakePicture(rospy.get_name())
    rospy.spin()

