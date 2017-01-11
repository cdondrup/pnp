#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_engage_human.msg import FindInteractantAction, FindInteractantResult
from pnp_msgs.msg import ActionResult
from pepper_move_base.msg import TrackPersonAction, TrackPersonGoal


class FindInteractant(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=FindInteractantAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        rospy.loginfo("Creating tracker client")
        self.start_client = SimpleActionClient("/start_tracking_person", TrackPersonAction)
        self.start_client.wait_for_server()
        rospy.loginfo("Tracker client connected")
        self._ps.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        rospy.loginfo("Finding interactant '%s'" % (goal.interactant_id,))
        self.start_client.send_goal(TrackPersonGoal(id=goal.id, interactant_id=goal.interactant_id, no_turn=True))
        res = FindInteractantResult()
        res.result.append(ActionResult(cond="found_interactant__"+goal.interactant_id+"__"+goal.id, truth_value=True))
        res.result.append(ActionResult(cond="free_interactant_id__"+goal.interactant_id, truth_value=False))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("find_interactant")
    FindInteractant(rospy.get_name())
    rospy.spin()

