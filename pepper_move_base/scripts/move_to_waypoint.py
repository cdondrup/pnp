#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pnp_msgs.msg import ActionResult
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pepper_move_base.msg import MoveToWaypointAction, MoveToWaypointResult
from geometry_msgs.msg import PoseStamped
from mongodb_store.message_store import MessageStoreProxy


class MoveToWaypoint(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = PNPSimplePluginServer(
            name,
            MoveToWaypointAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.msg_store = MessageStoreProxy(
            database=rospy.get_param("~db_name", "semantic_map"), 
            collection=rospy.get_param("~collection_name", "waypoints")
        )
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base client.")
        self.client.wait_for_server()
        self._as.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        pose, _ = self.load({"waypoint_name": goal.to})
        rospy.loginfo("Going from '%s' to '%s'" % (goal.from_, goal.to))
        self.client.send_goal_and_wait(MoveBaseGoal(target_pose=pose))
        res = MoveToWaypointResult()
        res.result.append(ActionResult(cond="robot_at_waypoint__"+goal.to, truth_value=True))
        res.result.append(ActionResult(cond="robot_at_waypoint__"+goal.from_, truth_value=False))
        res.result.append(ActionResult(cond="robot_pose_unknown", truth_value=True))
        self._as.set_succeeded(res)
        
    def load(self, meta):
        message = self.msg_store.query(PoseStamped._type, {}, meta)
        if len(message) == 0:
            raise Exception("Desired data set %s: %s not in datacentre."% meta.items()[0])
        else:
            return message[0][0], message[0][1]["_id"]


if __name__ == "__main__":
    rospy.init_node("move_to_waypoint")
    MoveToWaypoint(rospy.get_name())
    rospy.spin()

