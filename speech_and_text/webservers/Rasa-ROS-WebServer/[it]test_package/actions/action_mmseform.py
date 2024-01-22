from typing import Text,Any, Dict

from rasa_sdk import Tracker, FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict
import datetime

class ValidateMMSEForm(FormValidationAction):
    def name(self) -> Text:
        return "validate_mmse_form"

    #Counting wrong answers from the user
    wrong_answers = 0
    #Saving how many times the user answers correctly the pane_casa_gatto slot
    correct_pane_casa_gatto = 0

    def validate_current_year(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_year` value."""

        #Resetting global variables inside form class
        self.wrong_answers = 0
        self.correct_pane_casa_gatto = 0

        if (tracker.get_intent_of_latest_message() != "intent_define_current_year"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_year": "Wrong"}

        year = datetime.datetime.now().year
        message = slot_value.lower()
        if (year == 2022 and "duemila" in message and "ventidue" in message):
            dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
            return {"current_year": 2022}
        else: 
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_year": "Wrong"}
       
    
    def validate_current_season(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_season` value."""

        if (tracker.get_intent_of_latest_message() != "intent_define_current_season"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_season": "Wrong"}

        message = slot_value.lower()
        current_day = datetime.datetime.now()
        year = current_day.year

        winter_end = datetime.datetime(year, 3, 20)
        winter_start = None
        winter_end = None
        if (current_day.month == 12): 
            winter_start = datetime.datetime(year, 12, 21)
            winter_end = datetime.datetime(year + 1, 3, 20)
        else:
            winter_start = datetime.datetime(year - 1, 12, 21)
            winter_end = datetime.datetime(year, 3, 20)

        
        summer_start = datetime.datetime(year, 6, 21)
        summer_end = datetime.datetime(year, 9, 23)
        spring_start = datetime.datetime(year, 3, 20)
        spring_end = datetime.datetime(year + 1, 6, 21)
        fall_start = datetime.datetime(year, 9, 23)
        fall_end = datetime.datetime(year, 12, 21)

        if ("inverno" in message and current_day >= winter_start and current_day < winter_end):
                dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
                return {"current_season": "Inverno"}
        if ("estate" in message and current_day >= summer_start and current_day < summer_end):
                dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
                return {"current_season": "Estate"}
        if ("primavera" in message and current_day >= spring_start and current_day < spring_end):
                dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
                return {"current_season": "Primavera"}
        if ("autunno" in message and current_day >= fall_start and current_day < fall_end):
                dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
                return {"current_season": "Autunno"}
        else: 
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_season": "Wrong"}

    def validate_current_month(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_month` value."""

        if (tracker.get_intent_of_latest_message() != "intent_define_current_month"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_month": "Wrong"}

        months_dict = {
            1:"gennaio", 2:"febbraio", 3:"marzo", 4:"aprile", 5: "maggio", 6: "giugno", 7:"luglio",
            8: "agosto", 9:"settembre", 10: "ottobre", 11:"novembre", 12:"dicembre"
        }
        
        message = slot_value.lower()

        month = datetime.datetime.now().month

        if (months_dict[month] in message):
            dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
            return {"current_month": months_dict[month]}
        else:
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_month": "Wrong"}

    def validate_current_weekday(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_weekday` value."""

        if (tracker.get_intent_of_latest_message() != "intent_define_current_weekday"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_weekday": "Wrong"}

        #Necessary because italian vosk stt sometimes doesn't recognize 'ì'
        fixed_week_dict = {0:"lunedi", 1:"martedi", 2:"mercoledi", 3:"giovedi", 4:"venerdi", 5:"sabato", 6:"domenica"}
        
        #Correct answers for week_day slot
        week_dict = {0:"lunedì", 1:"martedì", 2:"mercoledì", 3:"giovedì", 4:"venerdì", 5:"sabato", 6:"domenica"}
        
        message = slot_value.lower()

        weekday = datetime.datetime.now().weekday()
        if (week_dict[weekday] in message or fixed_week_dict[weekday] in message):
            dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
            return {"current_weekday": week_dict[weekday]}
        else:
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_weekday": "Wrong"}

    def validate_current_country(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_country` value."""

        if (tracker.get_intent_of_latest_message() != "intent_define_current_country"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_country": "Wrong"}

        message = slot_value.lower()

        if ("italia" in message):
            dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
            return {"current_country": "italia"}
        else:
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_country": "Wrong"}
    
    def validate_current_region(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_country` value."""

        if (tracker.get_intent_of_latest_message() != "intent_define_current_region"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_region": "Wrong"}
        
        message = slot_value.lower()

        if ("lombardia" in message):
            dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
            return {"current_region": "lombardia"}
        else:
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"current_region": "Wrong"}

    def validate_tiger_vs_tiger(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `tiger_vs_tiger` value."""

        if (tracker.get_intent_of_latest_message() != "intent_say_tiger_vs_tiger"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"tiger_vs_tiger": "Wrong"}
        
        message = slot_value.lower()

        if ("tigre contro tigre" in message):
            dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
            return {"tiger_vs_tiger": True}
        else:
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            return {"tiger_vs_tiger": "Wrong"}


    def validate_pane_casa_gatto(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `pane_casa_gatto` value."""


        if (tracker.get_intent_of_latest_message() != "intent_say_pane_casa_gatto"):
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            if (self.correct_pane_casa_gatto == 1): dispatcher.utter_message(text=f"Hai ripetuto la sequenza di parole correttamente per una volta.")
            else: dispatcher.utter_message(text=f"Hai ripetuto la sequenza di parole correttamente per {self.correct_pane_casa_gatto} volte.")
            return {"pane_casa_gatto": self.correct_pane_casa_gatto}
        
        message = slot_value.lower()

        print(self.correct_pane_casa_gatto)

        if ("pane" in message and "casa" in message and "gatto" in message):
            self.correct_pane_casa_gatto += 1
            if (self.correct_pane_casa_gatto == 6):
                dispatcher.utter_message(text=f"Risposta corretta, andiamo avanti.")
                if (self.correct_pane_casa_gatto == 1): dispatcher.utter_message(text=f"Hai ripetuto la sequenza di parole correttamente per una volta.")
                else: dispatcher.utter_message(text=f"Hai ripetuto la sequenza di parole correttamente per {self.correct_pane_casa_gatto} volte.")
                return {"pane_casa_gatto": 6}
            repeat_times = 6 - self.correct_pane_casa_gatto
            if (repeat_times == 1): dispatcher.utter_message(text=f"Risposta corretta, ripeti ancora una volta.")
            else: dispatcher.utter_message(text=f"Risposta corretta, ripeti ancora {repeat_times} volte.")
            return {"pane_casa_gatto": None}
        else:
            dispatcher.utter_message(text=f"Va bene, proseguiamo con le domande.")
            self.wrong_answers += 1
            if (self.correct_pane_casa_gatto == 1): dispatcher.utter_message(text=f"Hai ripetuto la sequenza di parole correttamente per una volta.")
            else: dispatcher.utter_message(text=f"Hai ripetuto la sequenza di parole correttamente per {self.correct_pane_casa_gatto} volte.")
            return {"pane_casa_gatto": self.correct_pane_casa_gatto}

    def validate_reverse_counting(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `reverse_counting` value."""


        if (tracker.get_intent_of_latest_message() != "intent_define_reverse_counting"):
            self.wrong_answers += 1
            if (9 - self.wrong_answers == 1): dispatcher.utter_message(text=f"Hai risposto correttamente a una sola domanda.")
            else: dispatcher.utter_message(text=f"Hai risposto correttamente a { 9 - self.wrong_answers} domande.") 
            return {"reverse_counting": "Wrong"}
        
        values = list(tracker.get_latest_entity_values('number'))
        print(values)
        if (len(values) != 5):
            self.wrong_answers += 1
            if (9 - self.wrong_answers == 1): dispatcher.utter_message(text=f"Hai risposto correttamente a una sola domanda.")
            else: dispatcher.utter_message(text=f"Hai risposto correttamente a {9 - self.wrong_answers} domande.") 
            return {"reverse_counting": "Wrong"}

        answer_list = [93, 86, 79, 72, 65]

        for answer, current_value in zip(answer_list, values):
            if (answer == current_value): continue
            self.wrong_answers += 1
            if (9 - self.wrong_answers == 1): dispatcher.utter_message(text=f"Hai risposto correttamente a una sola domanda.")
            else: dispatcher.utter_message(text=f"Hai risposto correttamente a {9 - self.wrong_answers} domande.") 
            return {"reverse_counting": "Wrong"}

        dispatcher.utter_message(text=f"Risposta corretta.")
        if (9 - self.wrong_answers == 1): dispatcher.utter_message(text=f"Hai risposto correttamente a una sola domanda.")
        else: dispatcher.utter_message(text=f"Hai risposto correttamente a {9 - self.wrong_answers} domande.")
        return {"reverse_counting": True}
