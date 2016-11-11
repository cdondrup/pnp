#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_speak.msg import SpeakAction, SpeakResult
from pnp_msgs.msg import ActionResult
from std_msgs.msg import String


class Speak(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.texts = {
            "hello": "Hello, I am pepper!"        
        }
        self.predicate = rospy.get_param("~predicate", "said")
        self.pub = rospy.Publisher("/animated_speech", String, queue_size=10)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=SpeakAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
        self.pub.publish(self.texts[goal.text])
        rospy.sleep(3)
        res = SpeakResult()
        res.result.append(ActionResult(cond=self.predicate+"__"+goal.id+"__"+goal.text, truth_value=ActionResult.TRUE))
        self._ps.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("say")
    Speak(rospy.get_name())
    rospy.spin()

