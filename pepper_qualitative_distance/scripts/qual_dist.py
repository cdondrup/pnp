#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from nao_interaction_msgs.msg import PersonDetectedArray
from pepper_qualitative_distance.msg import QualitativeDistance, QualitativeDistanceArray
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv  import GetDomainPredicateDetailsService
import yaml


class QualitativeDistancePublisher(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.people = set([])
        self.predicate = rospy.get_param("~predicate", "robot_distance")        
        with open(rospy.get_param("~config_file"),'r') as f:        
            self.config = yaml.load(f)
        print self.config
        rospy.Subscriber("/naoqi_driver_node/people_detected", PersonDetectedArray, self.callback)
        self.pub = rospy.Publisher("~distances", QualitativeDistanceArray, queue_size=10)
        rospy.loginfo("... done")
        
    def callback(self, msg):
        a = QualitativeDistanceArray()
        a.header = msg.header
        people = []
        for p in msg.person_array:
            d = QualitativeDistance()
            d.id = p.id
            people.append(p.id)
            false_predicates = []
            true_predicates = []
            for dist in self.config["distances"]:
                if p.person.distance > float(dist["min"]) and p.person.distance < float(dist["max"]):
                    d.distance = dist["name"]
                    true_predicates.append(self.predicate+"__id_"+str(d.id)+"__"+dist["name"])
                    continue
                false_predicates.append(self.predicate+"__id_"+str(d.id)+"__"+dist["name"])
            a.distance_array.append(d)
        self.update_knowledgebase(false_predicates,0)
        self.update_knowledgebase(true_predicates,1)
        self.pub.publish(a)
        for x in people: self.people.add(x)
        self.decay(people)
        
    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        srv_name = "/kcl_rosplan/update_knowledge_base_array"
        req = KnowledgeUpdateServiceArrayRequest()
        req.update_type = req.ADD_KNOWLEDGE if truth_value else req.REMOVE_KNOWLEDGE
        predicate = [predicate] if not isinstance(predicate,list) else predicate
        for p in predicate:
            cond = p.split("__")
            rospy.logdebug("Updating %s %s" % (str(p), str(truth_value)))
            tp = self._get_predicate_details(cond[0]).predicate.typed_parameters
            if len(tp) != len(cond[1:]):
                rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (cond[0], len(tp), len(cond[1:])))
                return
            
            if truth_value:
                req.knowledge.append(KnowledgeItem(
                    knowledge_type=KnowledgeItem.INSTANCE,
                    instance_type="id",
                    instance_name=cond[1]
                ))
    
                self.__call_service(
                    srv_name,
                    KnowledgeUpdateServiceArray,
                    req
                )
            
            req.knowledge.append(KnowledgeItem(
                knowledge_type=KnowledgeItem.FACT,
                attribute_name=cond[0],
                values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(tp, cond[1:])]
            ))

            self.__call_service(
                srv_name,
                KnowledgeUpdateServiceArray,
                req
            )
                
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

    def decay(self, people):
        false_predicates = []
        for p in set(self.people)-set(people):
            self.people.remove(p)
            for dist in self.config["distances"]:
                false_predicates.append(self.predicate+"__id_"+str(p)+"__"+dist["name"])
        self.update_knowledgebase(false_predicates,0)
                
if __name__ == "__main__":
    rospy.init_node("qualitative_distance")
    q = QualitativeDistancePublisher(rospy.get_name())
    rospy.spin()

