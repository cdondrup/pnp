#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient
from pepper_move_base.msg import TrackPersonAction, TrackPersonGoal
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_engage_human.msg import TerminateInteractionAction, TerminateInteractionResult
from pnp_msgs.msg import ActionResult


class TerminateInteraction(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=TerminateInteractionAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        rospy.loginfo("Creating tracker client")
        self.stop_client = SimpleActionClient("/stop_tracking_person", TrackPersonAction)
        self.stop_client.wait_for_server()
        rospy.loginfo("Tracker client connected")
        self._ps.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
        rospy.loginfo("Disengaging human '%s'" % (goal.id,))
        self.stop_client.send_goal(TrackPersonGoal())
        res = TerminateInteractionResult()
        res.result.append(ActionResult(cond="engaged__"+goal.interactant_id+"__"+goal.text, truth_value=False))
        res.result.append(ActionResult(cond="said__"+goal.id+"__"+goal.text, truth_value=False))
        res.result.append(ActionResult(cond="found_interactant__"+goal.interactant_id+"__"+goal.id, truth_value=False))
        res.result.append(ActionResult(cond="human_exists__"+goal.id, truth_value=False))
        res.result.append(ActionResult(cond="free_interactant_id__"+goal.interactant_id, truth_value=True))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("terminate_interaction")
    TerminateInteraction(rospy.get_name())
    rospy.spin()

