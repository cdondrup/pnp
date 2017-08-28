#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from actionlib import SimpleActionClient
from dialogue_task_actions.msg import GiveVoucherAction, GiveVoucherGoal
from pepper_engage_human.msg import PromoteProductAction, PromoteProductResult
from pnp_msgs.msg import ActionResult


class PromoteProduct(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=PromoteProductAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.client = SimpleActionClient("/give_voucher", GiveVoucherAction)
        rospy.logdebug("Waiting for give voucher server")
        self.client.wait_for_server()
        self._ps.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        rospy.loginfo("promoting product '%s'" % (goal.product_id,))
        self.client.send_goal_and_wait(GiveVoucherGoal(product=goal.product_id))
        res = PromoteProductResult()
        res.result.append(ActionResult(cond="free_promotion_id__%s" % goal.interactant_id, truth_value=True))
        res.result.append(ActionResult(cond="promoted_product__%s" % goal.product_id, truth_value=True))
        res.result.append(ActionResult(cond="found_promotion__%s__%s" % (goal.interactant_id, goal.id), truth_value=False))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("promote_product")
    PromoteProduct(rospy.get_name())
    rospy.spin()

