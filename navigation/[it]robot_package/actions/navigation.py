#! /usr/bin/env python2

"""
TELEOPERATION MODE FILE
"""

from threading import Thread, Event
from time import sleep
from rasa_sdk import Action
import rospy
from geometry_msgs.msg import Twist
from rasa_sdk.events import FollowupAction, SlotSet

#Process to continuosly send data to cmd_vel
process = None
#default linear velocity
default_vel = 0.2
#default angular velocity
default_rotation = 0.2
#actual linear velocity
vel = 0.0
#actual angular velocity
rotation = 0.0
#thread active
active = None
#robot moving
is_moving = False

def publish_twist():
    while(True): #TODO improve
        global vel, rotation, is_moving, active

        if vel > 0 :
            is_moving = True
        else: is_moving = False

        twist = Twist()

        if active.is_set():
            vel = 0
            rotation = 0
            pub.publish(twist)
            break
            #raise Exception('Halt request')

        twist.angular.z = rotation # rotation velocity 
        twist.linear.x = vel # velocity along x axis
        pub = rospy.Publisher('cmd_vel', Twist, latch=True, queue_size=1)
        pub.publish(twist)


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

        global vel, rotation
        vel = default_vel
        rotation = 0.0
        return []

class Sinistra(Action):
    def name(self):
        return "action_robot_sx"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        global rotation
        rotation = default_rotation
        
        return []

class Destra(Action):
    def name(self):
        return "action_robot_dx"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        global rotation
        rotation = -default_rotation
    
        return []

class Stop(Action):
    def name(self):
        return "action_robot_stop"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        global rotation, vel
        vel = 0
        rotation = 0
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

        global vel, rotation
        vel = -default_vel
        rotation = 0
        sleep(0.5)
        vel = 0

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
        global vel, rotation
        vel = 0
        rotation = 0
        return []


class EnterTeleopMode(Action):
    def name(self):
        return "action_enter_teleop"

    def run(self, dispatcher, tracker, domain):
        dispatcher.utter_message(response='utter_enter_navmode')
        try: 
            rospy.init_node('robot_joystick', anonymous=True)
        except:
            rospy.logerr('InitNode Exception')
            pass # node already active
        
        global process, active
        active = Event()
        process = Thread(target=publish_twist)
        process.daemon = True
        process.start()

        return []


class ExitCurrentMode(Action):
    def name(self):
        return "action_exit_mode"

    def run(self, dispatcher, tracker, domain):
        mod = tracker.get_slot('mode')
        if not mod: 
            dispatcher.utter_message(response='utter_default')
            return []

        dispatcher.utter_message(text=f'Uscita dalla modalità {mod}')

        if mod == 'telecomando':
            global active, process
            active.set()
            process.join()

            
        return [SlotSet('mode', None)]