## Struttura

Ad ogni skill è associata una modalità, a seconda della quale saranno disponibili diversi comandi : 

- Nella modalità **joystick** si possono usare i comandi vai, destra, sinistra, indietro, stop; usati per teleoperare il robot.

- La modalità **navigazione** permette di raggiungere una nuova stanza tra quelle salvate nel database (comando raggiungi stanza) o di aggiungere una nuova stanza ad esso (comando salva stanza).

- Infine la modalità **discorso** permette all'utente di dialogare con il robot chiedendo ad esempio che ora è.

in tutte le modalità è presente un comando aggiuntivo "esci" per uscire dalla modalità corrente.

RASA non supporta direttamente questa suddivisione, pertanto è utilizzato uno slot ("mode"): quando viene riconoscito l'intento dell'utente di entrare in una certa modalità è eseguita la regola che associa il nome della skill scelta allo slot.

Se l'utente pronuncia qualcosa che RASA riconosce come un comando (anche in una modalità non coerente con esso), viene eseguita l'azione corrispondente: come prima cosa ogni azione deve controllare che sia valido nella modalità corrente, comparando il valore dello slot "mode" con il nome della skill a cui il comando appartiene; nel caso corrispondano viene eseguito, altrimenti viene restituito un feedback con Larynx: "comando non valido in questa modalità".

## Aggiungere nuova skill

### Step necessari

- Introdurre nuovo valore che "mode" può avere, aggiungendo nel file domani.yml una nuova tripla di valori alla sezione mappings dello slot.
    - type: from_intent
    - value: [nome_skill]
    - intent: [nome intento per entrare nella nuova skill]

- Aggiungere al file stories.yml o rule.yml una nuova storia/regola composta così:
    - intent: [nome intento per entrare nella nuova skill]
    - slot_was_set: 
        <br>mode : [nome skill]
    - action: [nome azione di feedback per utente]

- Inserire i nuovi comandi nel file rule.yml o stories.yml associando il nome dell'intento per attivare il comando all'azione che il comando deve eseguire: nel file .py sarà presente un controllo della modalità.