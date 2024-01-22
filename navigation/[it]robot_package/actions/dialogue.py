#! /usr/bin/env python2

from rasa_sdk import Action
from datetime import datetime, date

def check_mode(dispatcher, tracker):
    if tracker.get_slot('mode') != 'discorso': 
        dispatcher.utter_message(response='utter_invalid_command')
        return False

    return True

class WhatTime(Action):
    def name(self):
        return "action_get_time"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        time = datetime.now()
        dispatcher.utter_message(text=f'Sono le {time.hour} e {time.minute} minuti.')
        return []

class WhatDate(Action):
    def name(self):
        return "action_get_date"

    def run(self, dispatcher, tracker, domain):
        if not check_mode(dispatcher, tracker): return []

        settimana = ["lunedì", "martedì", "mercoledì", "giovedì", "venerdì", "sabato", "domenica"]
        mesi = ["gennaio", "febbraio", "marzo", "aprile", "maggio", "giugno", "luglio", "agosto", "settembre", "ottobre", "novembre", "dicembre"]
        
        dat = date.today().strftime("%d/%m/%Y").split("/")
        set = settimana[date.today().weekday()]
        dispatcher.utter_message(text=f'Oggi è {set} {int(dat[0])} {mesi[int(dat[1])-1]}')

        return []