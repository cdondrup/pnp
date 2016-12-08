#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import roslib
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from pepper_goal_server.msg import RouteDescriptionGoalServerAction
from rosplan_dispatch_msgs.msg import PlanAction, PlanGoal
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv  import GetDomainPredicateDetailsService
from diagnostic_msgs.msg import KeyValue
from std_srvs.srv import Empty, EmptyRequest
from pymongo import MongoClient


class GoalServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._as = SimpleActionServer(
            name,
            RouteDescriptionGoalServerAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        client = MongoClient(
            rospy.get_param("~db_host", "localhost"),
            int(rospy.get_param("~db_port", 62345))
        )
        self.db_name = rospy.get_param("~db_name", "semantic_map")
        self.collection_name = rospy.get_param("~collection_name", "idea_park")
        self.semantic_map_name = rospy.get_param("~semantic_map_name")
        self.db = client[self.db_name]
        self.client = SimpleActionClient("/kcl_rosplan/start_planning", PlanAction)
        self.client.wait_for_server()
        self._as.start()
        rospy.loginfo("... done")

    def execute_cb(self, goal):
        self.__call_service("/kcl_rosplan/clear_goals", Empty, EmptyRequest())
        result = self.db[self.collection_name].find_one(
            {
                "shop_id": str(goal.shop_id), 
                "semantic_map_name": self.semantic_map_name
            }
        )
#        self.add_goal("described_route__sid_%s__%s" % (goal.shop_id, result["waypoint"]))
#        if self.call_planning():
#            self.__call_service("/kcl_rosplan/clear_goals", Empty, EmptyRequest())
        self.remove_knowledge("finished_description__sid_%s__%s" % (goal.shop_id, result["waypoint"]))
        self.add_goal("finished_description__sid_%s__%s" % (goal.shop_id, result["waypoint"]))
        if self.call_planning():
            self._as.set_succeeded()
        self._as.set_aborted()
        
    def call_planning(self):
        while not self._as.is_preempt_requested() and not rospy.is_shutdown():
            if self.client.send_goal_and_wait(PlanGoal()) == GoalStatus.SUCCEEDED:
                return True
            rospy.sleep(0.2)
        return False
        
    def add_goal(self, goal):
        self.update_kb(goal, KnowledgeUpdateServiceArrayRequest.ADD_GOAL)
        
    def remove_knowledge(self, knowledge):
        self.update_kb(knowledge, KnowledgeUpdateServiceArrayRequest.REMOVE_KNOWLEDGE)
        
    def update_kb(self, predicate, update_type):
        srv_name = "/kcl_rosplan/update_knowledge_base_array"
        req = KnowledgeUpdateServiceArrayRequest()
        req.update_type = update_type
        predicate = [predicate] if not isinstance(predicate,list) else predicate
        for p in predicate:
            cond = p.split("__")
            rospy.loginfo("Adding %s" % str(p))
            tp = self._get_predicate_details(cond[0]).predicate.typed_parameters
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
    rospy.init_node("route_description_goal_server")
    GoalServer(rospy.get_name())
    rospy.spin()

