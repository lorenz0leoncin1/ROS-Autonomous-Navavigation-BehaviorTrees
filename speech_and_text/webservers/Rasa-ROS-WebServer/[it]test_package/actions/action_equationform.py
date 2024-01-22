from distutils.command.build_scripts import first_line_re
from typing import Text,Any, Dict

from rasa_sdk import Tracker, FormValidationAction, Action
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict
from random import randrange

first_number = 0
second_number = 0
operation = ''


class ActionGenerateRandomEquation(Action):
    def name(self):
        return "action_generate_random_equation"

    def run(self, dispatcher, tracker, domain):
        global first_number
        global second_number
        global operation
        first_number = randrange(10)
        second_number = randrange(10)
        operation = ['*', '+', '-'][randrange(3)]
        return []

class ActionAskEquationSolution(Action):
    def name(self):
        return "action_ask_equation_solution"

    def run(self, dispatcher, tracker, domain):
        global first_number
        global second_number
        global operation
        print(operation)
        if (operation == '*'): dispatcher.utter_message(text=f"Quanto fa {first_number} per {second_number}?")
        elif (operation == '+'): dispatcher.utter_message(text=f"Quanto fa {first_number} più {second_number}?")
        elif (operation == '-'): dispatcher.utter_message(text=f"Quanto fa {first_number} meno {second_number}?")
        return []


class ValidateEquationForm(FormValidationAction):
    def name(self) -> Text:
        return "validate_equation_form"

    def validate_equation_solution(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `equation` value."""

        global first_number
        global second_number
        global operation

        result = 0
        if (operation == '*'): result = first_number * second_number
        elif (operation == '+'): result = first_number + second_number
        elif (operation == '-'): result = first_number - second_number

        #No entities detected, answer doesn't contain any number
        if (len(tracker.latest_message['entities']) == 0):
            dispatcher.utter_message(f"Mi dispiace, ma non mi pare tu abbia detto un numero.")
            return {"equation_solution": "Wrong"}

        #Can't convert to int, given answer is not a number, closing the form
        if (tracker.latest_message['entities'][0]['entity'] != 'number'):
            dispatcher.utter_message(f"Mi dispiace, ma non mi pare tu abbia detto un numero.")
            return {"equation_solution": "Wrong"}

        answer_value = int(tracker.latest_message['entities'][0]['value'])

        print("User inserted answer: ", answer_value)
            
        if (result == answer_value): 
            dispatcher.utter_message(f"Bravo, il risultato è corretto.")
            return {"equation_solution":answer_value}
        else: 
            dispatcher.utter_message(f"Il risultato non è corretto, riprova.")
            return {"equation_solution":None}
