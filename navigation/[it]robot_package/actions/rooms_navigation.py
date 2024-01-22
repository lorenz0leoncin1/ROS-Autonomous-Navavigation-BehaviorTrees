#! /usr/bin/env python3

"""
ROOMS NAVIGATION MODE FILE
"""

from rasa_sdk import Action, FormValidationAction
import csv
import rospy
import actionlib
import logging
from speech_and_text.srv import Text
from geometry_msgs.msg import PoseWithCovarianceStamped
from rasa_sdk.events import SlotSet, ActiveLoop
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib_msgs.msg

mod_name = 'navigazione'

class ActionListRoom(Action):
    def name(self): 
        return "action_list_room"

    def run(self, dispatcher, tracker, domain):
        rooms = ""
        with open('rooms.csv', 'r') as f:
            rooms = ', '.join([y['Room'] for y in csv.DictReader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)])
        
        if len(rooms) == 0:
            dispatcher.utter_message(text='Nessuna stanza salvata')
        else:
            dispatcher.utter_message(text=f'Le stanze salvate sono: {rooms}.')
        return []


class ReachRoomValidation(FormValidationAction):
    def name(self):
        return "validate_go_to_room_form"

    def validate_room_name(self, slot_value, dispatcher, tracker, domain):
        with open('rooms.csv', 'r') as f:
            reader = csv.DictReader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            if slot_value in [y['Room'] for y in reader]:
                dispatcher.utter_message(text=f'Raggiungo la stanza {slot_value}')
                return {"room_name": slot_value}
            else:
                dispatcher.utter_message(text='Stanza non trovata')
                return {"room_name": None}

class ReachRoom(Action):
    def name(self):
        return "action_go_to_room"

    def run(self, dispatcher, tracker, domain):
        room_name = tracker.get_slot('room_name')


        params = {}
        
        # check if room_name is a regitered room
        logging.info("check if room_name is a registered room")
        with open('rooms.csv', 'r') as f:
            reader = csv.DictReader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)

            for row in reader:
                if room_name == row['Room']:
                    params = row


        try: 
            logging.info("Initializa ros node robot_navigation in anonymous mode")
            rospy.init_node('robot_navigation', anonymous=True)
        except: 
            rospy.loginfo('InitNode exception')
            pass # node already active

        #pause conversation 
        try:
            rospy.wait_for_service('handle_mic')
            hanlde_mic = rospy.ServiceProxy('handle_mic', Text)
            res = hanlde_mic("stop")

            if res.response: rospy.loginfo('Communication paused')
        except:
            rospy.logerr('Unable to pause communication')

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base server")
        client.wait_for_server()

        # define goal from params
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = params['Pos_x']
        goal.target_pose.pose.position.y = params['Pos_y']
        goal.target_pose.pose.orientation.x = params['Or_x']
        goal.target_pose.pose.orientation.y = params['Or_y']
        goal.target_pose.pose.orientation.z = params['Or_z']
        goal.target_pose.pose.orientation.w = params['Or_w']

        client.send_goal(goal)
        client.wait_for_result()

        #goal reached?
        if client.get_state() != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            dispatcher.utter_message(text=f'Non possibile raggiungere la destinazione')
            rospy.logerr('Destination not reachable')
        else: 
            dispatcher.utter_message(text=f'Destinazione raggiunta')
            rospy.loginfo('Destination reached')

        return [SlotSet('room_name', None)]


class HandleGoTo(Action):
    def name(self):
        return "action_handle_go_to_room"

    def run(self, dispatcher, tracker, domain):
        if tracker.get_slot('mode') != mod_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []

        return [ActiveLoop('go_to_room_form')]

