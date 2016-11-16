#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from nao_interaction_msgs.msg import PersonDetectedArray
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv  import GetDomainPredicateDetailsService
import yaml
from operator import attrgetter


class QualitativeDistancePublisher(object):
    __tp = {}
    
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.__last_request = {
            0: KnowledgeUpdateServiceArrayRequest(),
            1: KnowledgeUpdateServiceArrayRequest()
        }
        self.people = set([])
        self.predicate = rospy.get_param("~distance_predicate", "robot_distance")
        self.__update_srv_name = rospy.get_param("~update_srv_name","/kcl_rosplan/update_knowledge_base_array")
        self.__get_details_srv_name = rospy.get_param("~get_details_srv_name", "/kcl_rosplan/get_domain_predicate_details")
        with open(rospy.get_param("~distance_config_file"),'r') as f:        
            self.distance_config = yaml.load(f)
        print self.distance_config
        with open(rospy.get_param("~parsing_config_file"),'r') as f:        
            self.parsing_config = yaml.load(f)
        print self.parsing_config
        rospy.Subscriber("/naoqi_driver_node/people_detected", PersonDetectedArray, self.callback)
        rospy.loginfo("... done")
        
    def callback(self, msg):
        people = []
        for person in msg.person_array:
            true_predicates = []
            false_predicates = []
            instances = []
            for i in self.parsing_config["instances"].items():
                arg = tuple(a["format"]%str(attrgetter(a["attribute"])(person)) for a in i[1])                
                instances.append((i[0],arg))
            
            for p in self.parsing_config["predicates"].items():
                print p
                arg = []
                tv = []
                if isinstance(p[1], dict):
                    arg.append(self.__get_arguments(p[1]["arguments"], person))
                    tv.append(p[1]["truth_value"])
                elif isinstance(p[1], list):
                    for element in p[1]:
                        arg.append(self.__get_arguments(element["arguments"], person))
                        tv.append(element["truth_value"])
                        
                for x, y in zip(tv, arg):
                    truth_value = eval(x["comparison"].replace("$attribute", str(attrgetter(x["attribute"])(person))))
                    if truth_value:                
                        true_predicates.append((p[0],y))
                    else:
                        false_predicates.append((p[0],y))
            
#            for dist in self.distance_config["distances"]:
#                if person.person.distance > float(dist["min"]) and person.person.distance < float(dist["max"]):
#                    true_predicates.append((self.predicate, ("id_"+str(person.id) ,dist["name"])))
#                    continue
#                false_predicates.append((self.predicate, ("id_"+str(person.id) ,dist["name"])))
            
        self.update_knowledgebase(predicates=true_predicates, instances=instances)
        self.update_knowledgebase(predicates=false_predicates, truth_value=0)
        for x in people: self.people.add(x)
        self.decay(people)
        
    def __get_arguments(self, arguments, person):
        res = []        
        for a in arguments:
            try:
                res.append(a["format"]%str(attrgetter(a["attribute"])(person)))
            except KeyError:
                res.append(a["format"]%str(a["value"]))
        return tuple(res)
        
    def update_knowledgebase(self, predicates=None, instances=None, truth_value=1):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        req = KnowledgeUpdateServiceArrayRequest()
        req.update_type = req.ADD_KNOWLEDGE if truth_value else req.REMOVE_KNOWLEDGE
        
        if instances != None:
            instances = [instances] if not isinstance(instances,list) else instances
            for i in instances:
                rospy.loginfo("Updating %s %s" % (str(i), str(truth_value)))
                for n in i[1]:
                    req.knowledge.append(KnowledgeItem(
                        knowledge_type=KnowledgeItem.INSTANCE,
                        instance_type=i[0],
                        instance_name=n
                    ))

        if predicates != None:
            predicates = [predicates] if not isinstance(predicates,list) else predicates
            for p in predicates:
                rospy.loginfo("Updating %s %s" % (str(p), str(truth_value)))
                if not p[0] in self.__tp:
                    self.__tp[p[0]] = self._get_predicate_details(p[0]).predicate.typed_parameters
                if len(self.__tp[p[0]]) != len(p[1]):
                    rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (p[0], len(self.__tp[p[0]]), len(p[1])))
                    return
                
                req.knowledge.append(KnowledgeItem(
                    knowledge_type=KnowledgeItem.FACT,
                    attribute_name=p[0],
                    values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(self.__tp[p[0]], p[1])]
                ))
    
        if req.knowledge:
            if not self.__compare_knowledge_items(self.__last_request[truth_value].knowledge, req.knowledge):
                self.__call_service(
                    self.__update_srv_name,
                    KnowledgeUpdateServiceArray,
                    req
                )
            self.__last_request[truth_value] = req
                    
    def __call_service(self, srv_name, srv_type, req):
        print "CALLING SERVICE"
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
        return self.__call_service(
            self.__get_details_srv_name,
            GetDomainPredicateDetailsService,
            name
        )

    def decay(self, people):
        false_instances = []
        for p in set(self.people)-set(people):
            self.people.remove(p)
            for dist in self.distance_config["distances"]:
                false_instances.append(("id", ("id_"+str(p),)))
        self.update_knowledgebase(instances=false_instances, truth_value=0)
        
    def __compare_knowledge_items(self, k1, k2):
        if len(k1) != len(k2): return False
        for e1, e2 in zip(k1, k2):
            if e1 != e2: return False
        return True
        
if __name__ == "__main__":
    rospy.init_node("qualitative_distance")
    q = QualitativeDistancePublisher(rospy.get_name())
    rospy.spin()

