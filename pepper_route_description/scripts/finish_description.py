#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pnp_msgs.msg import ActionResult
from pepper_route_description.msg import FinishRouteDescriptionAction, FinishRouteDescriptionResult


class FinishDescription(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = PNPSimplePluginServer(
            name,
            FinishRouteDescriptionAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        res = FinishRouteDescriptionResult()
        res.result.append(ActionResult(cond="described_route__%s__%s" % (goal.shop_id,goal.waypoint), truth_value=False))
        res.result.append(ActionResult(cond="finished_description__%s__%s" % (goal.shop_id,goal.waypoint), truth_value=True))
        self._as.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("finish_description")
    FinishDescription(rospy.get_name())
    rospy.spin()

