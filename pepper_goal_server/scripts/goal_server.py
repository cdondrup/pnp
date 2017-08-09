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
    def __init__(self, name, predicate, parameters):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(
            name,
            GoalServerAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.planning_goal = predicate + "__" + "__".join(parameters) if len(parameters) else predicate
        self.client = SimpleActionClient("/kcl_rosplan/start_planning", PlanAction)
        self.client.wait_for_server()
        self._as.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        self.add_goal(self.planning_goal)
        tries = goal.tries if goal.tries > 1 else float("inf")
        cnt = 0
        while self.client.send_goal_and_wait(PlanGoal()) != GoalStatus.SUCCEEDED \
                and cnt < tries \
                and not self._as.is_preempt_requested() \
                and not rospy.is_shutdown():
            self._as.publish_feedback(GoalServerFeedback(cnt))
            rospy.sleep(goal.timeout)
            self.add_goal(self.planning_goal)
            cnt += 1
        else:
            self._as.set_aborted()
            return
        self._as.set_succeeded()
        
    def add_goal(self, goal):
        gu.clear_all()
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
                predicate=v["predicate"], 
                parameters=v["parameters"]
            )
        )
    rospy.spin()

