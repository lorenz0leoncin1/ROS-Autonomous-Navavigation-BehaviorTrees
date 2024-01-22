#! /usr/bin/env python2

from typing import Any, Dict, List
from rasa_sdk import Action, FormValidationAction
import csv
import rospy
from speech_and_text.srv import Text
from geometry_msgs.msg import PoseWithCovarianceStamped
from rasa_sdk.events import SlotSet, ActiveLoop, FollowupAction
import os
import logging

mod_create_name = 'crea routine'
mod_addroom_name = 'addroom routine'
class ActionSaveRoomToRoutine(Action):
    def name(self):
        return "action_save_routine"

    def run(self, dispatcher, tracker, domain):
        # Verifica che il sistema sia nella modalità "impara routine"
        mode = tracker.get_slot('mode')
        if mode != mod_addroom_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []

        # Nome della stanza da salvare
        room_name = tracker.latest_message.get("text")

        # Path dei file
        room_csv_path = 'rooms.csv'
        routine_csv_path = 'routine.csv'

        # Verifica se il file routine.csv è vuoto a partire dalla seconda riga
        is_routine_empty = self.is_routine_empty(routine_csv_path)

        # Scrivi i dati della stanza nel file routine.csv
        success = self.write_room_to_routine(room_name, room_csv_path, routine_csv_path)

        if success:
            dispatcher.utter_message(text=f'La stanza "{room_name}" è stata salvata nella routine.')
        #else:
            #dispatcher.utter_message(text='Errore durante il salvataggio della stanza nella routine.')
        return []

    def is_routine_empty(self, routine_csv_path):
        # Leggi il file routine.csv per verificare se è vuoto a partire dalla seconda riga
        try:
            with open(routine_csv_path, 'r') as routine_file:
                lines = routine_file.readlines()
                return len(lines) <= 1  # Il file è vuoto a partire dalla seconda riga
        except FileNotFoundError:
            # Se il file routine.csv non esiste, consideralo vuoto
            return True

    def write_room_to_routine(self, room_name, room_csv_path, routine_csv_path):
        # Scrivi i dati della stanza nel file routine.csv
        try:
            with open(routine_csv_path, 'a', newline='') as routine_file:
                # Apri il file room.csv e leggi i dati della stanza da lì
                with open(room_csv_path, 'r') as room_file:
                    reader = csv.reader(room_file)
                    for row in reader:
                        if row and row[0] == room_name:
                            writer = csv.writer(routine_file)
                            writer.writerow(row)
                            return True
            return False
        except Exception as e:
            #print(f"Errore durante il salvataggio della stanza nella routine: {e}")
            return False



class ActionHandleSave(Action):
    def name(self):
        return "action_handle_save_routine"

    def run(self, dispatcher, tracker, domain):
        # Verifica che il sistema sia nella modalità "impara"
        if tracker.get_slot('mode') != mod_create_name:
            dispatcher.utter_message(response='utter_invalid_command')
            
            return []

        # Verifica se il file routine.csv esiste e non è vuoto
        if not self.is_routine_empty():
            dispatcher.utter_message(text='Sto per sovrascrivere la routine precedente. Devo procedere?')
            return [SlotSet('mode', mod_create_name)]
        dispatcher.utter_message(text='Nomina il nome di una stanza salvata se la vuoi aggiungere alla routine. Dii salva per salvare la routine oppure dii "elenca routine" per vedere come stai costruendo la routine!')
        return[FollowupAction('action_list_room'), SlotSet('mode', mod_addroom_name)]
        
    def is_routine_empty(self):
        # Verifica se routine.csv esiste e non è vuoto
        if not os.path.exists('routine.csv'):
            return True

        with open('routine.csv', 'r') as routine_file:
            reader = csv.reader(routine_file)
            next(reader)  # Salta l'intestazione
            for row in reader:
                if row:
                    return False
        return True

#SETUPPA LA ROUTINE
class ActionSetupRoutine(Action):
    def name(self):
        return "action_setup_routine"

    def run(self, dispatcher, tracker, domain):
        # Verifica che il sistema sia nella modalità "impara"
        if tracker.get_slot('mode') != mod_create_name:
            previous_user_input = tracker.latest_message.get('text', None)
            if previous_user_input != "":
                dispatcher.utter_message(response='utter_invalid_command')
            return []

        last_msg_user = tracker.latest_message
        last_text_user = last_msg_user.get('text')

        logging.info(last_msg_user.get('text'))

        if last_text_user == "no":
            dispatcher.utter_message(text='Routine non sovrascritta')
            return [SlotSet('mode', None)]
            
        # Pulisci il file routine.csv mantenendo solo l'intestazione
        lines = [""]
        with open('routine.csv', 'r') as f:
            lines.append(f.readline())

        with open('routine.csv', 'w') as f:
            f.writelines(lines)

        nlu_yaml_path = "data/nlu.yml"
        intent_name = "intent_new_room"

        self.remove_nlu_rows(nlu_yaml_path, intent_name)

        #dispatcher.utter_message(text='Routine eliminata. Ora puoi crearne una nuova!')

        room_csv_file = 'rooms.csv'
        nlu_output_file = 'data/nlu.yml'

        # Genera un unico intent con tutti i nomi delle stanze come esempi
        try:
            room_names = self.get_room_names(room_csv_file)
            self.generate_room_intent(nlu_output_file, room_names)
            dispatcher.utter_message(text='Nomina il nome di una stanza salvata se la vuoi aggiungere alla routine. Dii salva per salvare la routine oppure dii "elenca routine" per vedere come stai costruendo la routine')
            return [FollowupAction('action_list_room'), SlotSet('mode', mod_addroom_name)]
        except FileNotFoundError as e:
            #dispatcher.utter_message(text='Errore: Il file room.csv non è stato trovato.')
            return [SlotSet('mode', None)]

    def remove_nlu_rows(self, nlu_yaml_path, intent_name):
        with open(nlu_yaml_path, 'r') as file:
            nlu_data = file.readlines()

        new_nlu_data = []
        remove_lines = False

        for line in nlu_data:
            if remove_lines:
                if line.strip() == "- ":
                    remove_lines = False
            elif line.strip() == f"- intent: {intent_name}":
                remove_lines = True
            else:
                new_nlu_data.append(line)

        with open(nlu_yaml_path, 'w') as file:
            file.writelines(new_nlu_data)

    # Funzione per contare le righe in un file CSV
    def count_csv_rows(self, file_path):
        with open(file_path, 'r') as file:
            return len(file.readlines()) - 1  # Sottrai 1 per escludere l'intestazione
            
    def is_routine_empty(self):
        # Verifica se routine.csv esiste e non è vuoto
        if not os.path.exists('routine.csv'):
            return True

        with open('routine.csv', 'r') as routine_file:
            reader = csv.reader(routine_file)
            next(reader)  # Salta l'intestazione
            for row in reader:
                if row:
                    return False
        return True

    def get_room_names(self, room_csv_path):
        room_names = []
        with open(room_csv_path, 'r') as room_file:
            reader = csv.DictReader(room_file)
            for row in reader:
                room_name = row['Room']
                room_names.append(room_name)
        return room_names

    def generate_room_intent(self, output_nlu_path, room_names):
        if room_names:
            with open(output_nlu_path, 'a') as nlu_file:
                nlu_file.write("- intent: intent_new_room\n")
                nlu_file.write("  examples: |\n")
                for room_name in room_names:
                    nlu_file.write(f"    - {room_name}\n")

#SALVA
class ActionDone(Action):
    def name(self):
        return "action_done"

    def run(self, dispatcher, tracker, domain):
        if tracker.get_slot('mode') != mod_addroom_name:
            dispatcher.utter_message(response='utter_invalid_command')
            return []
        dispatcher.utter_message('Routine salvata con successo!')
        return [SlotSet('mode', None)]

class ActionAskRoom(Action):
    def name(self):
        return "action_ask_room"

    def run(self, dispatcher, tracker, domain):
        if tracker.get_slot('mode') != mod_addroom_name:
            return []
        dispatcher.utter_message(response='utter_ask_room')
        return [FollowupAction('action_list_room')]           ######################################NEWWWWWWWWWWWWWWWWWWWW
    
#LISTA ROUTINE
class ActionListRoutine(Action):
    def name(self): 
        return "action_list_routine"

    def run(self, dispatcher, tracker, domain):
        rooms = ""
        with open('routine.csv', 'r') as f:
            rooms = ', '.join([row['Room'] for row in csv.DictReader(f, delimiter=',', quoting=csv.QUOTE_MINIMAL)])
        
        if len(rooms) == 0:
            dispatcher.utter_message(text='Routine vuota!')
        else:
            dispatcher.utter_message(text=f'La routine è composta dalle destinazioni seguenti: {rooms}.')
        return []
    

class ActionEnterRoutine(Action):
    def name(self):
        return "action_enter_impara_routine"
    def run(self, dispatcher, tracker, domain):
        dispatcher.utter_message(response='utter_enter_impara_routine')
        return []
