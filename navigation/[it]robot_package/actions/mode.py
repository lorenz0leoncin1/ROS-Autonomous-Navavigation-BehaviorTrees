#! /usr/bin/env python2

from rasa_sdk import Action
from rasa_sdk.events import SlotSet

class CheckMode(Action):
    def name(self):
        return "action_check_mode"

    def run(self, dispatcher, tracker, domain):
        mode = tracker.get_slot("mode")

        if mode == None:
            dispatcher.utter_message(text="Sei nel menu principale.")
        else:
            dispatcher.utter_message(text=f"Sei nella modalita: {mode}")

        return []
