#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient
from pepper_move_base.msg import TrackPersonAction, TrackPersonGoal
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from pepper_speak.msg import SpeakAction, SpeakResult
from pnp_msgs.msg import ActionResult
from std_msgs.msg import String
from mummer_dialogue.msg import dialogueAction, dialogueGoal


class Speak(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.texts = {
            "hello": "Hello, I am pepper!"        
        }
        self.predicate = rospy.get_param("~predicate", "said")
        rospy.loginfo("Starting dialogue client")
        self.client = SimpleActionClient(
            "/dialogue_start",
            dialogueAction
        )
        rospy.loginfo("Waiting for dialogue client")
        self.client.wait_for_server()
        rospy.loginfo("Dialogue client started")
        self.pub = rospy.Publisher("/animated_speech", String, queue_size=10)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=SpeakAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        rospy.loginfo("Creating tracker client")
        self.start_client = SimpleActionClient("/start_tracking_person", TrackPersonAction)
        self.start_client.wait_for_server()
        rospy.loginfo("Tracker client connected")
        self._ps.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
#        self.pub.publish(self.texts[goal.text])
#        rospy.sleep(3)
        self.start_client.send_goal(TrackPersonGoal(id=goal.id))
        self.client.send_goal(dialogueGoal(userID=int(goal.id.split("_")[1])))
        res = SpeakResult()
        res.result.append(ActionResult(cond=self.predicate+"__"+goal.id+"__"+goal.text, truth_value=ActionResult.TRUE))
        self._ps.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("say")
    Speak(rospy.get_name())
    rospy.spin()

