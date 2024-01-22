#! /usr/bin/env python2

from rasa_sdk import Action
from rasa_sdk.events import AllSlotsReset, SlotSet
from geometry_msgs.msg import Twist

import rospy

class ActionRobotReset(Action):
    def name(self):
        return "action_reset_all_slots"

    def run(self, dispatcher, tracker, domain):
        return [AllSlotsReset()]