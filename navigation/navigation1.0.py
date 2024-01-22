#! /usr/bin/env python2

"""
TELEOPERATION MODE FILE
"""

from time import sleep
from rasa_sdk import Action
import rospy
from geometry_msgs.msg import Twist
from rasa_sdk.events import FollowupAction

default_vel = 0.2
default_rotation = 0.2 #rad/s
vel = 0.0 # linear velocity
is_moving = False

def publish_twist(direction, angle, message):
    try: 
        rospy.init_node('robot_joystick', anonymous=True)
    except:
        rospy.logerr('InitNode Exception')
        pass # node already active

    global vel
    vel = direction

    global is_moving
    if vel > 0 :
        is_moving = True
    else: is_moving = False

    twist = Twist()
    twist.angular.z = angle # rotation velocity 
    twist.linear.x = direction # velocity along x axis
    pub = rospy.Publisher('cmd_vel', Twist, latch=True, queue_size=1)
    pub.publish(twist)

    rospy.loginfo(message)
    return

# check if user said a valid command in joystick mode
def check_mode(dispatcher, tracker) -> bool:
    # correct mode?
    if tracker.get_slot('mode') != 'telecomando': 
        dispatcher.utter_message(response='utter_invalid_command')
        return False

    return True

class Avanti(Action):
    def name(self):
        return "action_robot_avanti"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        publish_twist(default_vel, 0.0, 'Moving forward') # forward with no rotation
        return []

class Sinistra(Action):
    def name(self):
        return "action_robot_sx"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        global vel
        publish_twist(vel, default_rotation, 'Turning left') # 0.2 rad left
        
        return []

class Destra(Action):
    def name(self):
        return "action_robot_dx"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        global vel
        publish_twist(vel, -default_rotation, 'Turning right') # 0.2 rad right
    
        return []

class Stop(Action):
    def name(self):
        return "action_robot_stop"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        publish_twist(0.0, 0.0, 'Stop robot')
        return []

class Indietro(Action):
    def name(self):
        return "action_robot_indietro"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        global is_moving
        if is_moving: 
            dispatcher.utter_message(text='Per andare indietro fermare prima il robot')
            return []

        publish_twist(-default_vel, 0.0, 'Moving back') # backward of 0.2 m/s * 0.5s = 0.1 m
        sleep(0.5)
        publish_twist(0.0, 0.0, 'Stop robot')

        return []

class UtterCommands(Action):
    def name(self):
        return "action_utter_commands"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []
        return [FollowupAction('utter_navigation_commands')]

class RobotShutDown(Action):
    def name(self):
        return "action_shut_down"

    def run(self, dispatcher, tracker, domain):
        publish_twist(0.0, 0.0, 'Shut Down')