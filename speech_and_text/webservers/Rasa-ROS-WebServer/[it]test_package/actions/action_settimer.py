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
            dispatcher.utter_message(text=f"Non ho capito la durata del timer, puoi ripetere?")
            return []
        reminder = None
        print(entities)
        if (entities['entity'] != "duration"):
            dispatcher.utter_message(text=f"Non ho capito la durata del timer, puoi ripetere?")
            return []
        else:
            print("Valore numerico: {}".format(entities['additional_info']['value']))
            print("Unità di misura: {}".format(entities['additional_info']['unit']))
            seconds = entities['additional_info']['normalized']['value']
            print("Valore normalizzato in secondi: {}".format(seconds))
            dispatcher.utter_message(text=f"Ho impostato un timer di {entities['text']}.")

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