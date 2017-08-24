#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient
from pepper_move_base.msg import TrackPersonAction, TrackPersonGoal
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_move_base.msg import ChargeRobotAction, ChargeRobotResult
from pnp_msgs.msg import ActionResult
from nao_interaction_msgs.srv import Recharge, RechargeRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nao_interaction_msgs.msg import BatteryInfo


class ChargeRobot(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=ChargeRobotAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        rospy.loginfo("Creating tracker client")
        self.stop_client = SimpleActionClient("/stop_tracking_person", TrackPersonAction)
        self.stop_client.wait_for_server()
        rospy.loginfo("Tracker client connected")
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base client.")
        self.client.wait_for_server()
        rospy.loginfo("Move base client connected")
        self._ps.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        self.stop_client.send_goal(TrackPersonGoal())

        self.__call_service("/naoqi_driver/recharge/go_to_station", Recharge, RechargeRequest())

        while not rospy.is_shutdown():
            if rospy.wait_for_message("/naoqi_driver_node/battery", BatteryInfo).charging:
                break
            rospy.sleep(1.)

        print "Done"

        res = ChargeRobotResult()
        res.result.append(ActionResult(cond="charging__%s" % goal.interactant_id, truth_value=True))
        res.result.append(ActionResult(cond="not_charging__%s" % goal.interactant_id, truth_value=False))

        self._ps.set_succeeded(res)

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
    rospy.init_node("charge_robot")
    ChargeRobot(rospy.get_name())
    rospy.spin()

