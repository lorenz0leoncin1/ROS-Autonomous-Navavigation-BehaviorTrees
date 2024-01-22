#! /usr/bin/env python2

from typing import Any, Dict, List
from rasa_sdk import Action, FormValidationAction
import csv
import rospy
from speech_and_text.srv import Text
from geometry_msgs.msg import PoseWithCovarianceStamped
from rasa_sdk.events import SlotSet, ActiveLoop, FollowupAction

mod_name = 'impara'

class ValidateRoomNameForm(FormValidationAction):
    def name(self):
        return "validate_room_name_form"


    def validate_room_name(self, slot_value, dispatcher, tracker, domain):        
        # check if room_name is already used
        with open('rooms.csv', 'r') as f:
            reader = csv.DictReader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            if slot_value in [y['Room'] for y in reader]:
                dispatcher.utter_message(text='Nome già in uso')
                return {"room_name": None}

        return {"room_name": slot_value}


class SetNewRoom(Action):
    def name(self):
        return "action_save_room"

    def run(self, dispatcher, tracker, domain):
        room_name = tracker.get_slot('room_name')

        # exception when init_node is already active
        try:
            rospy.init_node('robot_navigation', anonymous=True)
        except:
            print('InitNode exception') # node already active
            pass

        # get current position
        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose
        
        # write new Room - Position - Orientation
        new_row = f'\n"{room_name}",{current_pose.position.x},{current_pose.position.y},{current_pose.position.z},{current_pose.orientation.x},{current_pose.orientation.y},{current_pose.orientation.z},{current_pose.orientation.w}'
        with open('rooms.csv', 'a') as f:
            f.write(new_row)

        dispatcher.utter_message(text=f'Salvata nuova stanza con nome {room_name}')

        return [SlotSet('room_name', None)]


class HandleSave(Action):
    def name(self):
        return "action_handle_save_room"

    def run(self, dispatcher, tracker, domain):
        if tracker.get_slot('mode') != mod_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []

        return [ActiveLoop('room_name_form')]

class DeleteRoom(Action):
    def name(self):
        return "action_delete_room"

    def run(self, dispatcher, tracker, domain):
        room_name ='"' + tracker.get_slot('room_name') + '"'
        
        with open('rooms.csv', 'r') as f:
            l1 = f.readlines()
            lines = [y.strip() for y in l1 if room_name not in y] #list without room_name line
            if len(lines) != len(l1):
                dispatcher.utter_message(text=f'Stanza {room_name} rimossa')
            else: 
                dispatcher.utter_message(text=f'Stanza {room_name} non trovata')

        with open('rooms.csv', 'w') as f:
            for line in lines:

                #add newline before every line except the first ones
                if lines.index(line) != 0:
                    f.write('\n')

                f.write(line)


        return [SlotSet('room_name', None)]

class HandleDeleteRoom(Action):
    def name(self):
        return "action_handle_delete_room"

    def run(self, dispatcher, tracker, domain):
        if tracker.get_slot('mode') != mod_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []

        return [ActiveLoop('delete_room_form')]

class HandleList(Action):
    def name(self) -> Text:
        return "action_handle_list"

    def run(self, dispatcher, tracker, domain) -> List[Dict[Text, Any]]:
        if tracker.get_slot('mode') != mod_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []

        return [FollowupAction('action_list_room')]