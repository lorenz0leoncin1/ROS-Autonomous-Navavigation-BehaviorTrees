from typing import Text,Any, Dict

from rasa_sdk import Tracker, FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.types import DomainDict
import datetime


class ValidateMMSEForm(FormValidationAction):
    def name(self) -> Text:
        return "validate_mmse_form"

    def validate_current_year(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_year` value."""

        print(tracker.latest_message['entities'])

        year = datetime.datetime.now().year
        message = slot_value.lower()
        if (year == 2022 and (("two thousand" in message and "twenty two" in message) or "twenty twenty two" in message)):
            dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
            return {"current_year": 2022}
        else: 
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"current_year": None}
       
    
    def validate_current_season(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_season` value."""

        print(tracker.latest_message['entities'])

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

        if ("winter" in message and current_day >= winter_start and current_day < winter_end):
                dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
                return {"current_season": "Winter"}
        if ("summer" in message and current_day >= summer_start and current_day < summer_end):
                dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
                return {"current_season": "Summer"}
        if ("spring" in message and current_day >= spring_start and current_day < spring_end):
                dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
                return {"current_season": "Spring"}
        if (("fall" in message or "autumn" in message) and current_day >= fall_start and current_day < fall_end):
                dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
                return {"current_season": "Fall"}
        else: 
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"current_season": None}

    def validate_current_month(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_month` value."""

        print(tracker.latest_message['entities'])


        months_dict = {
            1:"january", 2:"february", 3:"march", 4:"april", 5: "may", 6: "june", 7:"july",
            8: "august", 9:"september", 10: "october", 11:"november", 12:"december"
        }
        
        message = slot_value.lower()

        month = datetime.datetime.now().month

        if (months_dict[month] in message):
            dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
            return {"current_month": months_dict[month]}
        else:
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"current_month": None}

    def validate_current_weekday(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_weekday` value."""

        print(tracker.latest_message['entities'])


        week_dict = {0:"monday", 1:"tuesday", 2:"friday", 3:"thursday", 4:"friday", 5:"saturday", 6:"sunday"}
        
        message = slot_value.lower()

        weekday = datetime.datetime.now().weekday()
        if (week_dict[weekday] in message):
            dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
            return {"current_weekday": week_dict[weekday]}
        else:
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"current_weekday": None}

    def validate_current_country(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_country` value."""

        print(tracker.latest_message['entities'])

        
        message = slot_value.lower()

        if ("italy" in message):
            dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
            return {"current_country": "italy"}
        else:
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"current_country": None}
    
    def validate_current_region(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `current_country` value."""

        print(tracker.latest_message['entities'])

        
        message = slot_value.lower()

        if ("lombardy" in message or "lombardi" in message):
            dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
            return {"current_region": "lombardy"}
        else:
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"current_region": None}

    def validate_tiger_vs_tiger(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        """Validate `tiger_vs_tiger` value."""

        print(tracker.latest_message['entities'])

        
        message = slot_value.lower()

        if ("black background brown background" in message):
            dispatcher.utter_message(text=f"Perfect, let's move on to the next question.")
            return {"tiger_vs_tiger": True}
        else:
            dispatcher.utter_message(text=f"This doesn't seem to be the correct answer.")
            return {"tiger_vs_tiger": None}
