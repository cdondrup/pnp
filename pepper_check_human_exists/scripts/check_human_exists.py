#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_check_human_exists.msg import CheckHumanExistsAction, CheckHumanExistsResult
from pnp_msgs.msg import ActionResult
from nao_interaction_msgs.msg import PersonDetectedArray


class CheckHumanExistance(PNPSimplePluginServer):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.predicate = rospy.get_param("~predicate", "human_exists")
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=CheckHumanExistsAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
        rospy.loginfo("checking existance of '%s'" % (goal.id,))
        res = CheckHumanExistsResult()
        try:
            msg = rospy.wait_for_message("/naoqi_driver_node/people_detected", PersonDetectedArray, timeout=5.)
        except rospy.ROSException as e:
            rospy.logwarn(e)
            res.result.append(ActionResult(cond=self.predicate+"__"+goal.id, truth_value=ActionResult.FALSE))
            res.result.append(ActionResult(cond="found_interactant__"+goal.interactant_id+"__"+goal.id, truth_value=True))
            res.result.append(ActionResult(cond="free_interactant_id__"+goal.interactant_id, truth_value=False))
        else:
            found = ActionResult.FALSE
            int_id = int(goal.id.split("_")[1])
            for p in msg.person_array:
                if p.id == int_id:
                    found = ActionResult.TRUE
                    break
            res.result.append(ActionResult(cond=self.predicate+"__"+goal.id, truth_value=found))
        finally:
            if self._ps.is_preempt_requested():
                self._ps.set_preempted()
            elif res.result[-1].truth_value:
                self._ps.set_succeeded(res)
            else:
                print "##### ABORTED #####"
                self._ps.set_aborted(res)

if __name__ == "__main__":
    rospy.init_node("check_human_existance")
    CheckHumanExistance(rospy.get_name())
    rospy.spin()

