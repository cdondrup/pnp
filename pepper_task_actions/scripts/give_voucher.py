#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer
from pepper_task_actions.msg import GiveVoucherAction
from nao_interaction_msgs.srv import BehaviorManagerControl, BehaviorManagerControlRequest
from nao_interaction_msgs.srv import BehaviorManagerInfo, BehaviorManagerInfoRequest
from nao_interaction_msgs.srv import Say, SayRequest
from pymongo import MongoClient


class GiveVoucher(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(name, GiveVoucherAction, self.execute_cb, auto_start=False)
        self.logo_app = rospy.get_param("~logo_app", "showmummerlogo-a897b8/behavior_1")
        client = MongoClient(
            rospy.get_param("~db_host", "localhost"),
            int(rospy.get_param("~db_port", 62345))
        )
        self.db_name = rospy.get_param("~db_name", "semantic_map")
        self.db = client[self.db_name]
        self.collection_name = rospy.get_param("~collection_name", "idea_park")
        self.semantic_map_name = rospy.get_param("~semantic_map_name")
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
        result = self.db[self.collection_name].find_one(
            {
                "shop_id": str(goal.shop_id), 
                "semantic_map_name": self.semantic_map_name
            }
        )
        text = "%s currenty has %s on offer. Please use your phone to receive the voucher from my screen." % (result["shopName"],result["voucher_offer"])
        if self.__is_behaviour_running(self.logo_app):
            self.__enable_behaviour(self.logo_app, False)
        self.__enable_behaviour(result["voucher_app"])
        self.__say(text)
        rospy.sleep(5.)
        self.__enable_behaviour(result["voucher_app"], False)
        self.__enable_behaviour(self.logo_app)
        self._as.set_succeeded()
        
    def __enable_behaviour(self, name, flag=True):
        try:
            self.__call_service(
                "/naoqi_driver/behaviour_manager/start_behaviour" if flag else "/naoqi_driver/behaviour_manager/stop_behaviour", 
                BehaviorManagerControl, 
                BehaviorManagerControlRequest(name=name)
            )
        except:
            rospy.logwarn("Could not start logo app. Service responded with error.")
            
    def __is_behaviour_running(self, name):
        return name in self.__call_service(
            "/naoqi_driver/behaviour_manager/get_running_behaviors", 
            BehaviorManagerInfo, 
            BehaviorManagerInfoRequest()
        ).behaviors
        
    def __say(self, text):
        self.__call_service("/naoqi_driver/animated_speech/say", Say, SayRequest(text))

if __name__ == "__main__":
    rospy.init_node("give_voucher")
    GiveVoucher(rospy.get_name())
    rospy.spin()

