from rasa_sdk import Action
import datetime
from rasa_sdk.events import ReminderScheduled

class ActionSetTimer(Action):
    def name(self):
        return "action_set_timer"

    async def run(self, dispatcher, tracker, domain):
        try:
            entities = tracker.latest_message['entities'][0]    #Zero perchè mi interessa solo la prima entità trovata della durata
        except(IndexError):
            dispatcher.utter_message(text=f"I didn't get the timer length, can you repeat please?")
            return []
        reminder = None
        print(entities)
        if (entities['entity'] != "duration"):
            dispatcher.utter_message(text=f"I didn't get the timer length, can you repeat please?")
            return []
        else:
            print("Value: {}".format(entities['additional_info']['value']))
            print("Unit: {}".format(entities['additional_info']['unit']))
            seconds = entities['additional_info']['normalized']['value']
            print("Normalized value in seconds: {}".format(seconds))
            dispatcher.utter_message(text=f"Timer set for {entities['text']}.")

            #Definisco il momento dell'avviso
            date = datetime.datetime.now() + datetime.timedelta(seconds=seconds)
            
            reminder = ReminderScheduled(
                "EXTERNAL_timer",
                trigger_date_time=date,
                entities=tracker.latest_message['entities'],
                name="timer",
                kill_on_user_message=False,
            )

        return [reminder]