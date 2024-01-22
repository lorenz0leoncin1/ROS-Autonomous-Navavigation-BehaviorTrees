#! /usr/bin/env python3

"""
ROOMS ROUTINE MODE FILE
"""

from asyncio import CancelledError
import string
from rasa_sdk import Action, FormValidationAction
import csv
import rospy
import actionlib
import logging
from std_msgs.msg import Bool
from speech_and_text.srv import Text
from geometry_msgs.msg import PoseWithCovarianceStamped
from rasa_sdk.events import SlotSet, ActiveLoop
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import subprocess
from std_msgs.msg import String
import logging
mod_name = 'routine'




class ActionRoutine(Action):
    def name(self):
        return "action_routine"       
    
    def run(self, dispatcher, tracker, domain):
        try:
                # Inizializza il nodo ROS
            rospy.init_node('robot_routine', anonymous=True)

            # Pausa la conversazione (se necessario)
            try:
                rospy.wait_for_service('handle_mic')
                handle_mic = rospy.ServiceProxy('handle_mic', Text)
                res = handle_mic("stop")

                if res.response:
                    rospy.loginfo('Comunicazione in pausa')
            except:
                    rospy.logerr('Impossibile mettere in pausa la comunicazione')

                # Avvia il processo BT C++

            bt_executable = "rosrun behavior_tree_core prova_bt"
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", bt_executable])

            result = ""  # Inizializza result come una stringa vuota
            message =  rospy.wait_for_message("/topic_result_bt", String, timeout=6000)
            result = result + message.data  # Assegna il valore del messaggio a result
            dispatcher.utter_message(text=result)
            return []
        except CancelledError: 
            raise
        # Funzione per contare le righe di dati nel file "routine" escludendo l'intestazione     

  
        

class HandleStartRoutine(Action):
    def name(self):
        return "action_handle_start_routine"

    def run(self, dispatcher, tracker, domain):
        if tracker.get_slot('mode') != mod_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []

        rospy.loginfo('Attivo la modalità routine!')
        return [ActiveLoop('routine_form')] #da riempire con il form per la routine
