from rasa_sdk import Action
from datetime import datetime

class ActionUtterTime(Action):
    def name(self):
        return "action_utter_time"

    # Answers the with current time in hours:minutes
    def run(self, dispatcher, tracker, domain):
        now = datetime.now()
        current_time = now.strftime("Sono le %H e %M.")
        dispatcher.utter_message(text=f"{current_time}")
        return []