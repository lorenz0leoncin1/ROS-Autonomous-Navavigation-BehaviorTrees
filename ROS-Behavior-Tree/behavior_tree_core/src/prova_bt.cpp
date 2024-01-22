/*FILE MAIN DEL BEHAVIOR TREE*/

#include <ros/ros.h>
#include <behavior_tree.h>
#include <fstream>
#include <vector>
#include "actions/move_to_position.h" // Importa l'implementazione di MoveToPosition1
#include <iostream>
#include <thread>
#include <chrono>
int main(int argc, char **argv)
{
    std::this_thread::sleep_for(std::chrono::seconds(5));
    ros::init(argc, argv, "BehaviorTree");
    try
    {
        int TickPeriod_milliseconds = 1000;

        //Vettore nodi azione
        std::vector<BT::MoveToPosition1*> moveNodes;
        //Vettore nodi fallback
        std::vector<BT::FallbackNodeWithMemory*> fallbackNodes;
        //Vettore nodi fault
        std::vector<BT::FaultNodes*> faultNodes;
        //inserisci il percorso dove tu hai memorizzato il file
        std::ifstream routineFile("/home/lorenzoleoncini/catkin_ws/src/navigation/[it]robot_package/routine.csv");
       
        BT::SequenceNodeWithMemory* sequence1 = new BT::SequenceNodeWithMemory("seq1");


        // Verifica se il file è aperto
        int id = 1;  // Inizializza il primo ID
        if (routineFile.is_open()) {
            std::string line;
            

            // Prima riga del file (da saltare)
            std::getline(routineFile, line);
            ///CREARE LE ISTANZE

            while (std::getline(routineFile, line)) {

                fallbackNodes.push_back(new BT::FallbackNodeWithMemory("FB " + std::to_string(id)));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                moveNodes.push_back(new BT::MoveToPosition1("A " + std::to_string(id) , id));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));    
                faultNodes.push_back(new BT::FaultNodes("FN " + std::to_string(id) , id));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // Incrementa l'ID per la prossima riga
                id++;    
            }         
            // Chiudi il file
            routineFile.close();
        }
        int count = 0;
        //ASSEGNARE LE ISTANZE CON UN CICLO FOR



        for (int i = 0; i < moveNodes.size(); i++) {
            sequence1->AddChild(fallbackNodes[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            fallbackNodes[i]->AddChild(moveNodes[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            fallbackNodes[i]->AddChild(faultNodes[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            count++;
        }


        //std::cerr << "Numero totale di nodi nella sequenza: " << sequence1->GetChildrenNumber() << "\n" << std::endl;

        Execute(sequence1, TickPeriod_milliseconds);  // da BehaviorTree.cpp

        
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
