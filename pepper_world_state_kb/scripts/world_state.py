#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rostopic
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import GetDomainPredicateDetailsService, GetInstanceService
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
import yaml
from operator import attrgetter
from threading import Thread
from Queue import Queue


class PlanningWorldState(object):
    __tp = {}
    
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.__last_request = {}
        self.people = set([])
        self.predicate = rospy.get_param("~distance_predicate", "robot_distance")
        self.__update_srv_name = rospy.get_param("~update_srv_name","/kcl_rosplan/update_knowledge_base_array")
        self.__get_details_srv_name = rospy.get_param("~get_details_srv_name", "/kcl_rosplan/get_domain_predicate_details")
        self.__get_instances_srv_name = rospy.get_param("~get_instances_srv_name", "/kcl_rosplan/get_current_instances")
        with open(rospy.get_param("~config_file"),'r') as f:        
            config = yaml.load(f)
        rospy.loginfo("Inserting static instances.")
        self.update_knowledgebase(instances=self._create_instances(config["static_instances"]))
        rospy.loginfo("Inserting static predicates.")
        t,f = self._create_predicates(config["static_predicates"])
        self.update_knowledgebase(predicates=t)
        self.update_knowledgebase(predicates=f, truth_value=0)
        rospy.loginfo("Starting reset service")
        self.srv = rospy.Service("~reset_kb", Empty, lambda x:self.reset_cb(x, config))
        self.subscribers = Queue()
        for inputs in config["inputs"]:
            thread = Thread(target=self.subscribe, args=(inputs, self.subscribers))
            thread.start()
        rospy.loginfo("... done")

    def subscribe(self, inputs, sub_queue):
        rospy.loginfo("Subsribing to '%s'." % inputs["topic"])
        self.__last_request[inputs["topic"]] = {0: [], 1: []}
        sub_queue.put(
            rospy.Subscriber(
                name=inputs["topic"],
                data_class=rostopic.get_topic_class(inputs["topic"], blocking=True)[0],
                callback=self.callback,
                callback_args=inputs
            )
        )
        
    def reset_cb(self, msg, config):
        rospy.loginfo("Resetting knowledge base. Clearing current knowledge.")
        self.__call_service("/kcl_rosplan/clear_knowledge_base", Empty, EmptyRequest())
        for inputs in config["inputs"]:
            self.__last_request[inputs["topic"]] = {0: [], 1: []}
        rospy.loginfo("Inserting static instances.")
        self.update_knowledgebase(instances=self._create_instances(config["static_instances"]))
        rospy.loginfo("Inserting static predicates.")
        t,f = self._create_predicates(config["static_predicates"])
        self.update_knowledgebase(predicates=t)
        self.update_knowledgebase(predicates=f, truth_value=0)
        return EmptyResponse()
        
    def callback(self, msg, config):
        data_field = attrgetter(config["base_attribute"])(msg) if config["base_attribute"] != "" else msg
        data_field = [data_field] if not isinstance(data_field, list) else data_field
        
        true_predicates = []
        false_predicates = []
        instances = []
        for data in data_field:
            try:
                instances.extend(self._create_instances(config["data"]["instances"], data))
            except KeyError:
                rospy.logdebug("No instances to be created from message")
            
            try:
                t,f = self._create_predicates(config["data"]["predicates"], data)
            except KeyError:
                rospy.logdebug("No predicates to be created from message")
            else:
                true_predicates.extend(t)
                false_predicates.extend(f)
         
        self.update_knowledgebase(key=config["topic"], predicates=true_predicates, instances=instances)
        self.update_knowledgebase(key=config["topic"], predicates=false_predicates, truth_value=0)
        self.decay(instances)
        
    def _create_instances(self, data, msg=None):
        res = []
        for i in data.items():
            argument_list = [i[1]] if not isinstance(i[1],list) else i[1]
            for element in argument_list:
                res.append((i[0],self.__get_arguments(element["arguments"], msg)))
        return res
        
    def _create_predicates(self, data, msg=None):
        true_predicates = []
        false_predicates = []
        for p in data.items():
            arg = []
            tv = []
            argument_list = [p[1]] if not isinstance(p[1],list) else p[1]
            for element in argument_list:
                try:
                    arg.append(self.__get_arguments(element["arguments"], msg))
                except KeyError:
                    arg.append(tuple())
                tv.append(element["truth_value"])
                    
            for x, y in zip(tv, arg):
                try:
                    truth_value = eval(x["comparison"].replace("$attribute", str(attrgetter(x["attribute"])(msg))))
                except TypeError:
                    # Cannot make comparison, assuming static truth value
                    truth_value = bool(x)
                if truth_value:                
                    true_predicates.append((p[0],y))
                else:
                    false_predicates.append((p[0],y))
        return true_predicates, false_predicates
        
    def __get_arguments(self, arguments, data=None):
        res = []
        for a in arguments:
            try:
                if data == None:
                    raise KeyError
                res.append(a["format"]%str(attrgetter(a["attribute"])(data)))
            except KeyError:
                res.append(str(a["value"]))
        return tuple(res)
        
    def update_knowledgebase(self, key=None, predicates=None, instances=None, truth_value=1):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        req = KnowledgeUpdateServiceArrayRequest()
        req.update_type = req.ADD_KNOWLEDGE if truth_value == 1 else req.REMOVE_KNOWLEDGE
        
        if instances != None:
            instances = [instances] if not isinstance(instances,list) else instances
            for i in instances:
                rospy.logdebug("Updating %s %s" % (str(i), str(truth_value)))
                for n in i[1]:
                    req.knowledge.append(KnowledgeItem(
                        knowledge_type=KnowledgeItem.INSTANCE,
                        instance_type=i[0],
                        instance_name=n
                    ))

        if predicates != None:
            predicates = [predicates] if not isinstance(predicates, list) else predicates
            for p in predicates:
                rospy.logdebug("Updating %s %s" % (str(p), str(truth_value)))
                if not p[0] in self.__tp:
                    self.__tp[p[0]] = self._get_predicate_details(p[0]).predicate.typed_parameters
                if len(self.__tp[p[0]]) != len(p[1]):
                    rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (p[0], len(self.__tp[p[0]]), len(p[1]), p))
                    return
                
                req.knowledge.append(KnowledgeItem(
                    knowledge_type=KnowledgeItem.FACT,
                    attribute_name=p[0],
                    values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(self.__tp[p[0]], p[1])]
                ))
    
        if req.knowledge:
            try:
                if not self.__compare_knowledge_items(self.__last_request[key][truth_value], req.knowledge):
                    self.__call_service(
                        self.__update_srv_name,
                        KnowledgeUpdateServiceArray,
                        req
                    )
                self.__last_request[key][truth_value] = req.knowledge
                self.__check_last_req(req.knowledge, key, (truth_value-1)*-1)
            except KeyError:
                self.__call_service(
                    self.__update_srv_name,
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
        return self.__call_service(
            self.__get_details_srv_name,
            GetDomainPredicateDetailsService,
            name
        )

    def decay(self, instances):
        instance_types = set()
        current_instances = set()

        for x in instances: 
            instance_types.add(x[0])
            current_instances.add(x[1][0])
        false_instances = []

        for i in instance_types:
            db = set(self.__call_service(self.__get_instances_srv_name, GetInstanceService, i).instances)
            for to_del in db - current_instances:
                false_instances.append((i,(to_del,)))
        
        if false_instances:
            self.update_knowledgebase(instances=false_instances, truth_value=0)
        
    def __compare_knowledge_items(self, k1, k2):
        if len(k1) != len(k2): return False
        for e1, e2 in zip(k1, k2):
            if e1 != e2: return False
        return True
        
    def __check_last_req(self, new, key, truth_value):
        for e in new:
            if e in self.__last_request[key][truth_value]:
                self.__last_request[key][truth_value] = []
                return
        
if __name__ == "__main__":
    rospy.init_node("planning_world_state_manager")
    PlanningWorldState(rospy.get_name())
    rospy.spin()

