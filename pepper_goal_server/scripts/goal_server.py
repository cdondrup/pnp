#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rosplan_python_utils.knowledge_base_utils as kb
import rosplan_python_utils.goal_utils as gu
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from pepper_goal_server.msg import GoalServerAction, GoalServerFeedback
from rosplan_dispatch_msgs.msg import PlanAction, PlanGoal
import yaml


class GoalServer(object):
    def __init__(self, name, sub_goals):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(
            name,
            GoalServerAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.planning_goals = [x["predicate"] + "__" + "__".join(x["parameters"]) for x in sub_goals]
        self.client = SimpleActionClient("/kcl_rosplan/start_planning", PlanAction)
        self.client.wait_for_server()
        self._as.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        self.add_goals(self.planning_goals)
        tries = goal.tries if goal.tries > 1 else float("inf")
        cnt = 0
        while self.client.send_goal_and_wait(PlanGoal()) != GoalStatus.SUCCEEDED \
                and cnt < tries \
                and not self._as.is_preempt_requested() \
                and not rospy.is_shutdown():
            self._as.publish_feedback(GoalServerFeedback(cnt))
            rospy.sleep(goal.timeout)
            self.add_goals(self.planning_goals)
            cnt += 1

        if self._as.is_preempt_requested():
            self.client.cancel_all_goals()
            self._as.set_preempted()
        else:
            self._as.set_succeeded()
        
    def add_goals(self, goals):
        gu.clear_all()
        for goal in goals:
            kb.remove(goal)
            gu.add(goal)


if __name__ == "__main__":
    rospy.init_node("goal_server")
    with open(rospy.get_param("~config_file"), 'r') as f:
        config = yaml.load(f)
    servers = []
    for k, v in config.items():
        servers.append(
            GoalServer(
                name=rospy.get_name()+"/"+k, 
                sub_goals=v["sub_goals"]
            )
        )
    rospy.spin()

