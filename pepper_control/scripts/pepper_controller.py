#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import time
import rospy
import rostopic
import roslib
import yaml
from actionlib import SimpleActionClient
from pepper_control.cfg import PepperControlConfig
from dynamic_reconfigure.server import Server as DynServer
from rosplan_knowledge_msgs.srv import KnowledgeQueryService, KnowledgeQueryServiceRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv  import GetDomainPredicateDetailsService
from diagnostic_msgs.msg import KeyValue
from nao_interaction_msgs.srv import SetBreathEnabled, SetBreathEnabledRequest
from nao_interaction_msgs.srv import LocalizationTrigger, LocalizationTriggerRequest
from nao_interaction_msgs.srv import LocalizationTriggerString, LocalizationTriggerStringRequest
from nao_interaction_msgs.srv import LocalizationGetErrorMessage, LocalizationGetErrorMessageRequest
from nao_interaction_msgs.srv import LocalizationCheck, LocalizationCheckRequest
from nao_interaction_msgs.srv import GoToPosture, GoToPostureRequest
from nao_interaction_msgs.srv import BehaviorManagerControl, BehaviorManagerControlRequest
from nao_interaction_msgs.msg import PersonDetectedArray
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed 
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import String
from collections import OrderedDict
import numpy as np
from threading import Thread

CLIENT = "client"
GOAL = "goal"
ACTIONGOAL = "actiongoal"
PRECONDITION = "precondition"


class PepperController(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        rospy.on_shutdown(self.__on_shut_down)
        self.pnp_state = ""
        rospy.Subscriber("/rosplan_bridge/current_state", String, self.callback)
        DynServer(PepperControlConfig, self.dyn_callback)
        with open(rospy.get_param("~config_file"), 'r') as f:
            config = yaml.load(f)
        print config
        self.localisation_dir = rospy.get_param("~localisation_dir", "")
        self.logo_app = rospy.get_param("~logo_app", "showmummerlogo-a897b8/behavior_1")
        self.once = rospy.get_param("~once", False)
        self.clients = OrderedDict()
        self.client_probabilities = []
        for k, v in config.items():
            rospy.loginfo("Creating '%s' client" % v["topic"])
            self.clients[k] = {
                CLIENT: SimpleActionClient(v["topic"], self.get_action_type(v["topic"])),
                GOAL: v["goal"],
                ACTIONGOAL: self.get_goal_type(v["topic"])(*v["parameters"]),
                PRECONDITION: v["precondition"]
            }
            rospy.loginfo("Waiting for '%s' server" % v["topic"])
            self.clients[k][CLIENT].wait_for_server()
            self.client_probabilities.append(v["probability"])
            rospy.loginfo("Connected to '%s' server" % v["topic"])
        rospy.loginfo("... done")
        
    def callback(self, msg):
        print "received", msg
        self.pnp_state = msg.data
    
    def dyn_callback(self, config, level):
        self.pitch_range = config["pitch_range"]
        self.pitch_offset = config["pitch_offset"]
        self.yaw_range = config["yaw_range"]
        print config
        return config
        
    def get_goal_type(self, action_name):
        while not rospy.is_shutdown():
            action_name = action_name[1:] if action_name[0] == "/" else action_name
            topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
            # remove "Action" string from goal type
            try:
                assert("Action" in topic_type)
            except TypeError:
                rospy.logwarn("Topic %s doe not seem to be available yet, retrying in 1 second")
                rospy.sleep(1.)
                continue
            else:
                return roslib.message.get_message_class(topic_type[:-10]+"Goal")

    def get_action_type(self, action_name):
        while not rospy.is_shutdown():
            action_name = action_name[1:] if action_name[0] == "/" else action_name
            topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
            # remove "Goal" string from action type
            try:
                assert("Goal" in topic_type)
            except TypeError:
                rospy.logwarn("Topic %s doe not seem to be available yet, retrying in 1 second")
                rospy.sleep(1.)
                continue
            else:
                return roslib.message.get_message_class(topic_type[:-4])
    
    def _get_predicate_details(self, name):
        srv_name = "/kcl_rosplan/get_domain_predicate_details"
        while not rospy.is_shutdown():
            try:
                return self.__call_service(
                    srv_name,
                    GetDomainPredicateDetailsService,
                    name
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)
        
    def query_knowledgbase(self, predicate):
        """querry the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :return (int) -1 (unknown), 0 (false), or 1 (true)
        """
        cond = predicate.split("__")
        srv_name = "/kcl_rosplan/query_knowledge_base"
        tp = self._get_predicate_details(cond[0]).predicate.typed_parameters
        if len(tp) != len(cond[1:]):
            rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (cond[0], len(tp), len(cond[1:])))
            return
        req = KnowledgeQueryServiceRequest()
        req.knowledge.append(
            KnowledgeItem(
                knowledge_type=KnowledgeItem.FACT,
                attribute_name=cond[0],
                values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(tp, cond[1:])]
            )
        )
        while not rospy.is_shutdown():
            try:
                r = self.__call_service(
                    srv_name,
                    KnowledgeQueryService,
                    req
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)
            else:
                return 1 if r.all_true else 0
                
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
                
    def idle(self):
        j = JointAnglesWithSpeed()
        j.joint_names = ['HeadYaw', 'HeadPitch']
        j.joint_angles = [.5,-.5]
        j.speed = .05
        pub = rospy.Publisher("/joint_angles", JointAnglesWithSpeed, queue_size=10)
        while not rospy.is_shutdown() and self.is_idle:
            try:
#                rospy.sleep(5.)
#                raise rospy.ROSException
                rospy.wait_for_message("/naoqi_driver_node/people_detected", PersonDetectedArray, timeout=5.)
            except rospy.ROSException, rospy.ROSInterruptException:
                j.joint_angles = [(np.random.rand()-.5)*self.yaw_range, -(np.random.rand()*self.pitch_range+self.pitch_offset)]
                pub.publish(j)
                
    def localise_robot(self, ldir):
        if ldir != "":
            rospy.loginfo("Loading localisation data from: '%s'" % ldir)
            res = self.__call_service(
                "/naoqi_driver/localization/load",
                LocalizationTriggerString,
                LocalizationTriggerStringRequest(self.localisation_dir)
            )
            if res.result != 0:
                rospy.logwarn(self.__get_error_message(res.result))
            else:
                return
        
        self.__localise(ldir)
                        
    def __localise(self, ldir):
        ldir = ldir if ldir != "" else time.strftime("loc_%Y_%m_%d_%H_%M_%S")
        rospy.loginfo("Starting localisation")
        res = self.__call_service(
            "/naoqi_driver/localization/learn_home",
            LocalizationTrigger,
            LocalizationTriggerRequest()
        )
        if res.result != 0:
            rospy.logwarn(self.__get_error_message(res.result))
        if not self.__call_service("/naoqi_driver/localization/is_data_available", LocalizationCheck, LocalizationCheckRequest()).result:
            rospy.logerr("No localisation data available")
            sys.exit(1)
        else:
            rospy.loginfo("Saving localisation data to: '%s'" % ldir)
            res = self.__call_service(
                "/naoqi_driver/localization/save",
                LocalizationTriggerString,
                LocalizationTriggerStringRequest(ldir)
            )
            if res.result != 0:
                rospy.logwarn(self.__get_error_message(res.result))
                
    def __get_error_message(self, code):
        return self.__call_service(
            "/naoqi_driver/localization/get_message_from_error_code",
            LocalizationGetErrorMessage,
            LocalizationGetErrorMessageRequest(code)
        ).error_message
        
    def set_breathing(self, flag):
        self.__call_service(
            "/naoqi_driver/motion/set_breath_enabled", 
            SetBreathEnabled, 
            SetBreathEnabledRequest(SetBreathEnabledRequest.ARMS, flag)
        )
        
    def wake_up(self):
        self.__call_service(
            "/naoqi_driver/motion/wake_up", 
            Empty, 
            EmptyRequest()
        )
        
    def rest(self):
        self.__call_service(
            "/naoqi_driver/motion/rest", 
            Empty, 
            EmptyRequest()
        )
        
    def stand(self):
        self.__call_service(
            "/naoqi_driver/robot_posture/go_to_posture", 
            GoToPosture, 
            GoToPostureRequest(GoToPostureRequest.STAND_INIT, 0.5)
        )
        
    def show_logo(self, flag):
        try:
            self.__call_service(
                "/naoqi_driver/behaviour_manager/start_behaviour" if flag else "/naoqi_driver/behaviour_manager/stop_behaviour", 
                BehaviorManagerControl, 
                BehaviorManagerControlRequest(name=self.logo_app)
            )
        except:
            rospy.logwarn("Could not start logo app. Service responded with error.")
    
    def start_idle(self):
        self.is_idle = True
        t = Thread(target=self.idle, args=())
        t.start()
        return t
        
    def stop_idle(self, thread):
        self.is_idle = False
        thread.join()
    
    def spin(self):
        self.wake_up()
        self.stand()
        self.show_logo(True)
        self.set_breathing(True)
        self.localise_robot(self.localisation_dir)
        cnt = 0
        while not rospy.is_shutdown():
            print self.once
            self.stand()
            action = np.random.choice(self.clients.keys(), p=self.client_probabilities)
            rospy.loginfo("Chose '%s' for execution" % action)
            rospy.loginfo("Checking precondition '%s'" % self.clients[action][PRECONDITION])
            while not rospy.is_shutdown() and not self.query_knowledgbase(self.clients[action][PRECONDITION]):
                rospy.sleep(1.0)
            if self.once and cnt >= 1: break
            rospy.loginfo("Starting execution of '%s'" % action)
            t = self.start_idle()
            self.pnp_state = ""
            while not rospy.is_shutdown() and self.pnp_state != "goal":
                rospy.loginfo("Executing '%s'" % action)
                self.clients[action][CLIENT].send_goal_and_wait(self.clients[action][ACTIONGOAL])
                rospy.sleep(1.0)
            self.stop_idle(t)
            rospy.loginfo("Waiting for goal state to be true")
            while not rospy.is_shutdown() and not self.query_knowledgbase(self.clients[action][GOAL]):
                rospy.sleep(1.0)
            rospy.loginfo("Goal '%s' achieved" % self.clients[action][GOAL])
            self.pnp_state = ""
            self.stand()
            cnt += 1
            
    def __on_shut_down(self):
        self.set_breathing(False)
        self.__call_service(
            "/naoqi_driver/localization/stop_all",
            Empty,
            EmptyRequest()
        )
        self.show_logo(False)
        self.rest()


if __name__ == "__main__":
    rospy.init_node("pepper_controller")
    p = PepperController(rospy.get_name())
    p.spin()
#    p.wake_up()
#    p.stand()
#    p.start_idle()
