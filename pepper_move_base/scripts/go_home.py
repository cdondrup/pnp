#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_move_base.msg import GoHomeAction, GoHomeResult
from pnp_msgs.msg import ActionResult
from nao_interaction_msgs.srv import LocalizationTrigger, LocalizationTriggerRequest
from nao_interaction_msgs.srv import LocalizationCheck, LocalizationCheckRequest
from nao_interaction_msgs.srv import LocalizationGetErrorMessage, LocalizationGetErrorMessageRequest
from std_srvs.srv import Empty, EmptyRequest


class GoHome(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=GoHomeAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.register_preempt_callback(self.preempt_cb)
        self._ps.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        if self.__call_service("/naoqi_driver/localization/is_data_available", LocalizationCheck, LocalizationCheckRequest()).result:
            while not rospy.is_shutdown() and not self._ps.is_preempt_requested():
                rospy.loginfo("Robot returning home")
                res = self.__call_service(
                    "/naoqi_driver/localization/go_to_home",
                    LocalizationTrigger,
                    LocalizationTriggerRequest()
                )
                if res.result != 0:
                    rospy.logwarn(self.__get_error_message(res.result))
                break
        else:
            rospy.logwarn("Cannot go home. Pepper has never been localised.")
            
        res = GoHomeResult()
        res.result.append(ActionResult(cond="robot_at_home", truth_value=True))
        res.result.append(ActionResult(cond="robot_pose_unknown", truth_value=False))
        self._ps.set_succeeded(res)
        
    def preempt_cb(self):
        rospy.loginfo("Preempting going home")
        self.__call_service(
            "/naoqi_driver/localization/stop_all",
            Empty,
            EmptyRequest()
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
                
    def __get_error_message(self, code):
        return self.__call_service(
            "/naoqi_driver/localization/get_message_from_error_code",
            LocalizationGetErrorMessage,
            LocalizationGetErrorMessageRequest(code)
        ).error_message


if __name__ == "__main__":
    rospy.init_node("go_home")
    GoHome(rospy.get_name())
    rospy.spin()

