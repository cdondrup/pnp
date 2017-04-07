#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from pepper_goal_server.msg import GoalServerAction, GoalServerFeedback
from rosplan_dispatch_msgs.msg import PlanAction, PlanGoal
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv  import GetDomainPredicateDetailsService
from diagnostic_msgs.msg import KeyValue
from std_srvs.srv import Empty, EmptyRequest
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
        self.planning_goal = predicate + "__" + "__".join(parameters)
        print self.planning_goal
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
        self.__call_service("/kcl_rosplan/clear_goals", Empty, EmptyRequest())
        self.__add_goal(self.planning_goal)
    
    def __add_goal(self, goal):
        srv_name = "/kcl_rosplan/update_knowledge_base_array"
        req = KnowledgeUpdateServiceArrayRequest()
        req.update_type = req.ADD_GOAL
        goal = [goal] if not isinstance(goal,list) else goal
        for p in goal:
            cond = p.split("__")
#            rospy.loginfo("Adding %s" % str(p))
            try:
                tp = self._get_predicate_details(cond[0]).predicate.typed_parameters
            except AttributeError as e:
                rospy.logwarn(e)
            else:
                if len(tp) != len(cond[1:]):
                    rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (cond[0], len(tp), len(cond[1:])))
                    return
                req.knowledge.append(KnowledgeItem(
                    knowledge_type=KnowledgeItem.FACT,
                    attribute_name=cond[0],
                    values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(tp, cond[1:])]
                ))

        while not rospy.is_shutdown():
            try:
                self.__call_service(
                    srv_name,
                    KnowledgeUpdateServiceArray,
                    req
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)
            else:
                return
                
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

    def _get_predicate_details(self, name):
        srv_name = "/kcl_rosplan/get_domain_predicate_details"
        while not rospy.is_shutdown():
            try:
                return self.__call_service(
                    srv_name,
                    GetDomainPredicateDetailsService,
                    name
                )
            except rospy.ROSInterruptException:
                rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
                rospy.sleep(1.)


if __name__ == "__main__":
    rospy.init_node("goal_server")
    with open(rospy.get_param("~config_file"), 'r') as f:
        config = yaml.load(f)
    servers = []
    for k,v in config.items():    
        servers.append(
            GoalServer(
                name=rospy.get_name()+"/"+k, 
                predicate=v["predicate"], 
                parameters=v["parameters"]
            )
        )
    rospy.spin()

