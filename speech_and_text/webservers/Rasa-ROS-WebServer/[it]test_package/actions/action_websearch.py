from rasa_sdk import Action
import requests
from lxml import html
import re


class ActionWebSearch(Action):
    def name(self):
        return "action_web_search"
    
    #Fixing Larynx TTS incapability to say numbers over 1000
    def larynx_numbers_fix_ita(self, msg):
        updated_msg = msg
        cifre = {0: "", 1: "uno", 2: "due", 3:"tre", 4:"quattro", 5:"cinque",
        6:"sei", 7: "sette", 8: "otto", 9:"nove"}
        decine = {0: "", 2: "venti", 3:"trenta", 4:"quaranta", 5:"cinquanta",
        6:"sessanta", 7: "settanta", 8: "ottanta", 9:"novanta"}
        words_list = msg.split()
        for index in range(0, len(words_list)):
            match = re.match('^[0-9]{4}', words_list[index])
            if (match): 
                new_word = ""
                data = match.group(0)
                numbers_list = [n for n in data]
                if (int(numbers_list[0]) == 1): new_word += "mille"
                else: new_word += cifre[int(numbers_list[0])] + "mila"
                if (int(numbers_list[1]) == 1): new_word += "cento"
                elif (int(numbers_list[1]) == 0): new_word += ""
                else: new_word += cifre[int(numbers_list[1])] + "cento"
                if (int(numbers_list[2]) == 1): 
                    if (int(numbers_list[3]) == 0): new_word += "dieci"
                    if (int(numbers_list[3]) == 1): new_word += "undici" 
                    if (int(numbers_list[3]) == 2): new_word += "dodici" 
                    if (int(numbers_list[3]) == 3): new_word += "tredici" 
                    if (int(numbers_list[3]) == 4): new_word += "quattordici" 
                    if (int(numbers_list[3]) == 5): new_word += "quindici" 
                    if (int(numbers_list[3]) == 6): new_word += "sedici" 
                    if (int(numbers_list[3]) == 7): new_word += "diciasette" 
                    if (int(numbers_list[3]) == 8): new_word += "diciotto" 
                    if (int(numbers_list[3]) == 9): new_word += "diciannove"
                    words_list[index] = new_word
                    updated_msg = ' '.join(words_list)
                else: 
                    new_word += decine[int(numbers_list[2])] 
                    new_word += cifre[int(numbers_list[3])] 
                    words_list[index] = new_word + "."
                    updated_msg = ' '.join(words_list)
        return updated_msg
        

    def run(self, dispatcher, tracker, domain):
        print("Found entities:\n" + str(tracker.latest_message['entities']))
        message = tracker.latest_message['text'].lower()
        query = message.replace("cerca su internet", "").strip()
        query = query.replace("cerca sul web", "").strip()
        query = query.replace("cerca su web", "").strip()
        query = query.replace("cerca", "").strip()
        
        print("La tua query è: " + query)

        headers = {'User-Agent': 'Mozilla/5.0 (Linux x86_64; rv:96.0) Gecko/20100101 Firefox/96.0'}
        try:
            page = requests.get('https://www.google.it/search?q={}'.format(query), headers=headers)
        except:
            dispatcher.utter_message("Mi dispiace, ma è impossibile stabilire una connessione internet.")
            return []
        tree = html.fromstring(page.content)
        wiki_path = tree.xpath('//*[@id="kp-wp-tab-overview"]/div[1]/div/div/div/div/div/div[1]/div/div/div/span[1]/text()')
        alternative_wiki_path = tree.xpath('//*[@id="kp-wp-tab-overview"]/div[1]/div/div/div/div/div/div[3]/div/div/div/div/span[1]/text()')
        msg = ""

        if(len(wiki_path) != 0):
                wiki_result = "{}".format(wiki_path[0]).strip()
                dispatcher.utter_message(text=f"Questo è ciò che ho trovato su Wikipedia.")
                msg = self.larynx_numbers_fix_ita(f"{wiki_result}")
        elif (len(alternative_wiki_path) != 0):
            wiki_result = "{}".format(alternative_wiki_path[0]).strip()
            dispatcher.utter_message(text=f"Questo è ciò che ho trovato su Wikipedia.")
            msg = self.larynx_numbers_fix_ita(f"{wiki_result}")
        else: msg = f"Mi dispiace, non ho trovato nulla sul web a riguardo."
        
        dispatcher.utter_message(text=msg)
        return []