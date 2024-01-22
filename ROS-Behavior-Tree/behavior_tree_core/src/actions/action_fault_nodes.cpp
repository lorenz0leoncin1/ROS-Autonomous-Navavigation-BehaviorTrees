#include <actions/move_to_position.h>
#include <string>
#include <behavior_tree.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <iostream>
#include <speech_and_text/Text.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>  // Per i messaggi booleani
#include "std_msgs/String.h"
#include "actions/move_to_position.h"

BT::FaultNodes::FaultNodes(std::string name, int id) : ActionNode::ActionNode(name), id_(id)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = false;
    time_ = 3;
    thread_ = std::thread(&FaultNodes::WaitForTick, this);
}

BT::FaultNodes::~FaultNodes() {}


// Funzione per verificare se il file 'room.csv' è vuoto
bool fault_is_room_csv_empty()
{
    
    // Percorso completo per il file 'rooms.csv'
    std::string percorso_file_csv = "/home/lorenzoleoncini/catkin_ws/src/navigation/[it]robot_package/routine.csv";

    // Apri il file in modalità lettura
    std::ifstream file(percorso_file_csv);

    // Verifica se il file è aperto e se è vuoto, dovrebbe ritornare 0 o 1 ???
    if (file.is_open())
    {
        // Leggi il contenuto del file
        std::string riga;
        while (std::getline(file, riga))
        {
            if (!riga.empty())
            {
                // Il file contiene dati, quindi non è vuoto
                file.close();
                return false;
            }
        }

        // Chiudi il file e restituisci true se è vuoto
        file.close();
        return true;
    }

    // Restituisci true se il file non può essere aperto (presumibilmente vuoto)
    return true;
}


bool fault_read_and_move_to_room(int id) {
    // Percorso completo per il file 'rooms.csv'
    std::string percorso_file_csv = "/home/lorenzoleoncini/catkin_ws/src/navigation/[it]robot_package/routine.csv";

    // Apri il file in modalità lettura
    std::ifstream file(percorso_file_csv);

    // Verifica se il file è aperto
    if (!file.is_open()) {
        std::cerr << "Errore: Impossibile aprire il file 'rooms.csv'." << std::endl;
        return false;
    }

    // Leggi la prima riga (che dovrebbe contenere le intestazioni delle colonne)
    std::string intestazioni;
    if (!std::getline(file, intestazioni)) {
        std::cerr << "Errore: Il file 'rooms.csv' non contiene intestazioni di colonna." << std::endl;
        file.close();
        return false;
    }

    // Leggi il file 'routine.csv' alla riga corrispondente all'ID
    std::string riga;

    for (int i = 0; i < id ; i++) {
        if (!std::getline(file, riga)) {
            std::cerr << "Errore: Impossibile trovare la riga con ID " << id << " in 'routine.csv'." << std::endl;
            file.close();
            return false;
        }
    }
    std::cerr << "Sto analizzando la riga " << id << std::endl;
    // Utilizza uno stringstream per suddividere la riga in token utilizzando ',' come delimitatore
    std::istringstream iss(riga);
    std::string token;
    std::vector<std::string> token_vettore;

    while (std::getline(iss, token, ',')) {
        token_vettore.push_back(token);
    }

    // Verifica se ci sono abbastanza token (almeno 8) prima di convertirli in valori double utilizzando std::stod()
    if (token_vettore.size() < 8) {
        std::cerr << "Errore: Dati insufficienti nel file 'rooms.csv'." << std::endl;
        file.close();
        return false;
    }

    // Converti direttamente i token in variabili per il goal di movimento
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // Verifica che il frame_id sia corretto
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = std::stod(token_vettore[1]);
    goal.target_pose.pose.position.y = std::stod(token_vettore[2]);
    goal.target_pose.pose.orientation.x = std::stod(token_vettore[3]);
    goal.target_pose.pose.orientation.y = std::stod(token_vettore[4]);
    goal.target_pose.pose.orientation.z = std::stod(token_vettore[5]);
    goal.target_pose.pose.orientation.w = std::stod(token_vettore[6]);

    // Chiudi il file
    file.close();

    // Crea un SimpleActionClient per l'azione "move_base"
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);

    // Attendere che il server "move_base" si avvii
    ROS_INFO("In attesa del server move_base...");
    move_base.waitForServer(ros::Duration(5.0));

    // Invia il goal al server "move_base"
    ROS_INFO("Invio del goal al server move_base...");
    move_base.sendGoal(goal);

    // Attendere il risultato del movimento
    move_base.waitForResult();

    bool result = false;
    //std::cerr << "IL TEMPO DI DURATA DELL'ESECUZIONE = "<< execution_duration << std::endl;
    
        // Verifica se il movimento è stato completato con successo
        if (move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Movimento completato con successo");
            result=true;
        } else {
            ROS_ERROR("Movimento fallito");
            move_base.cancelGoal();
        }   
    return result;
}

void BT::FaultNodes::WaitForTick()
{
    ROS_INFO("Retrying to send the robot to the expected position!");
    if(fault_is_room_csv_empty()){
        //break;
        set_status(BT::FAILURE);
        DEBUG_STDOUT("Azione " << get_name() << " FALLITA: Almeno un nodo azione ha fallito");
    }
    //while (true)
    //{
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);
        // Perform action...
        // Esegui l'azione...
        set_boolean_value(fault_read_and_move_to_room(this->id_));

        int i = 0;
        while (get_status() != BT::HALTED && i++ < time_)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id: " << std::this_thread::get_id());
            //std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (get_status() != BT::HALTED)
        {   
            //this->boolean_value_ = false;
        
            if (boolean_value_)
            {
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
                ROS_INFO("Movimento completato con successo");
                set_result_value(true);
                DEBUG_STDOUT(" VALORE RESULT_VALUE FAULT: " << result_value_ << " !");
            }
            else
            {
                
                DEBUG_STDOUT(" Routine FAILED! The robot is blocked or didn't reach the target!");
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
                set_result_value(false);
                DEBUG_STDOUT(" VALORE RESULT_VALUE FAULT: " << result_value_ << " !");
            }
        }
   // }
}

void BT::FaultNodes::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}


void BT::FaultNodes::set_time(int time)
{
    time_ = time;
}

void BT::FaultNodes::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}

bool BT::FaultNodes::get_boolean_value()
{
    return boolean_value_;
}