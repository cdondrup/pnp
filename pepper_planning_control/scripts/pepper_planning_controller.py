#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import time
import rospy
import rostopic
import roslib
import yaml
from actionlib import SimpleActionClient
from pepper_planning_control.cfg import PepperPlanningControlConfig
from dynamic_reconfigure.server import Server as DynServer
import rosplan_python_utils.knowledge_base_utils as kb
import pepper_python_utils.service_utils as service_utils
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
from actionlib_msgs.msg import GoalStatus
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
        with open(rospy.get_param("~config_file"), 'r') as f:
            config = yaml.load(f)
        print config
#        self.localisation_dir = rospy.get_param("~localisation_dir", "")
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
            print self.clients[k][ACTIONGOAL]
            rospy.loginfo("Waiting for '%s' server" % v["topic"])
            self.clients[k][CLIENT].wait_for_server()
            self.client_probabilities.append(v["probability"])
            rospy.loginfo("Connected to '%s' server" % v["topic"])
        DynServer(PepperPlanningControlConfig, self.dyn_callback)
        rospy.loginfo("... done")
        
    def callback(self, msg):
        print "received", msg
        self.pnp_state = msg.data
    
    def dyn_callback(self, config, level):
        self.pitch_range = config["pitch_range"]
        self.pitch_offset = config["pitch_offset"]
        self.yaw_range = config["yaw_range"]
        for i, (k, c) in enumerate(self.clients.items()):
            config["action_%s" % i] = k
            self.client_probabilities[i] = 1. if config["run_action_%s" % i] else 0.
            c[CLIENT].cancel_all_goals()
        self.client_probabilities = np.array(self.client_probabilities)*1./np.sum(self.client_probabilities)
        print self.client_probabilities
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
                
#    def localise_robot(self, ldir):
#        if ldir != "":
#            rospy.loginfo("Loading localisation data from: '%s'" % ldir)
#            res = service_utils.call_service(
#                "/naoqi_driver/localization/load",
#                LocalizationTriggerString,
#                LocalizationTriggerStringRequest(self.localisation_dir)
#            )
#            if res.result != 0:
#                rospy.logwarn(self.__get_error_message(res.result))
#            else:
#                return
#        
#        self.__localise(ldir)
                        
    def __localise(self, ldir):
        ldir = ldir if ldir != "" else time.strftime("loc_%Y_%m_%d_%H_%M_%S")
        rospy.loginfo("Starting localisation")
        res = service_utils.call_service(
            "/naoqi_driver/localization/learn_home",
            LocalizationTrigger,
            LocalizationTriggerRequest()
        )
        if res.result != 0:
            rospy.logwarn(self.__get_error_message(res.result))
        if not service_utils.call_service("/naoqi_driver/localization/is_data_available", LocalizationCheck, LocalizationCheckRequest()).result:
            rospy.logerr("No localisation data available")
            sys.exit(1)
        else:
            rospy.loginfo("Saving localisation data to: '%s'" % ldir)
            res = service_utils.call_service(
                "/naoqi_driver/localization/save",
                LocalizationTriggerString,
                LocalizationTriggerStringRequest(ldir)
            )
            if res.result != 0:
                rospy.logwarn(self.__get_error_message(res.result))
                
    def __get_error_message(self, code):
        return service_utils.call_service(
            "/naoqi_driver/localization/get_message_from_error_code",
            LocalizationGetErrorMessage,
            LocalizationGetErrorMessageRequest(code)
        ).error_message
        
    def set_breathing(self, flag):
        service_utils.call_service(
            "/naoqi_driver/motion/set_breath_enabled", 
            SetBreathEnabled, 
            SetBreathEnabledRequest(SetBreathEnabledRequest.ARMS, flag)
        )
        
    def wake_up(self):
        service_utils.call_service(
            "/naoqi_driver/motion/wake_up", 
            Empty, 
            EmptyRequest()
        )
        
    def rest(self):
        service_utils.call_service(
            "/naoqi_driver/motion/rest", 
            Empty, 
            EmptyRequest()
        )
        
    def stand(self):
        service_utils.call_service(
            "/naoqi_driver/robot_posture/go_to_posture", 
            GoToPosture, 
            GoToPostureRequest(GoToPostureRequest.STAND_INIT, 0.5)
        )
        
    def show_logo(self, flag):
        try:
            service_utils.call_service(
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
#        self.localise_robot(self.localisation_dir)
        cnt = 0
        while not rospy.is_shutdown():
            print self.once
            self.stand()
            action = np.random.choice(self.clients.keys(), p=self.client_probabilities)
            rospy.loginfo("Chose '%s' for execution" % action)
            rospy.loginfo("Checking precondition '%s'" % self.clients[action][PRECONDITION])
            while not rospy.is_shutdown() and not kb.query(self.clients[action][PRECONDITION]):
                rospy.sleep(1.0)
            if self.once and cnt >= 1: break
            rospy.loginfo("Starting execution of '%s'" % action)
            t = self.start_idle()
            self.pnp_state = ""
            while not rospy.is_shutdown() and self.pnp_state != "goal":
                rospy.loginfo("Executing '%s'" % action)
                self.clients[action][CLIENT].send_goal_and_wait(self.clients[action][ACTIONGOAL])
                rospy.sleep(1.0)
                if self.clients[action][CLIENT].get_state() == GoalStatus.PREEMPTED:
                    break
            self.stop_idle(t)
            if self.clients[action][CLIENT].get_state() == GoalStatus.PREEMPTED:
                rospy.loginfo("Goal server has been preempted.")
                continue
            rospy.loginfo("Waiting for goal state to be true")
            while not rospy.is_shutdown() and not kb.query(self.clients[action][GOAL]):
                rospy.sleep(1.0)
            rospy.loginfo("Goal '%s' achieved" % self.clients[action][GOAL])
            self.pnp_state = ""
            self.stand()
            cnt += 1
            
    def __on_shut_down(self):
        self.set_breathing(False)
#        service_utils.call_service(
#            "/naoqi_driver/localization/stop_all",
#            Empty,
#            EmptyRequest()
#        )
        self.show_logo(False)
        self.rest()


if __name__ == "__main__":
    rospy.init_node("pepper_planning_controller")
    p = PepperController(rospy.get_name())
    p.spin()
#    p.wake_up()
#    p.stand()
#    p.start_idle()
