
/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <sequence_node_with_memory.h>
#include <string>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h> 

BT::SequenceNodeWithMemory::SequenceNodeWithMemory(std::string name) : ControlNode::ControlNode(name)
{
    reset_policy_ = BT::ON_SUCCESS_OR_FAILURE;
    current_child_idx_ = 0;  // initialize the current running child
}


BT::SequenceNodeWithMemory::SequenceNodeWithMemory(std::string name, int reset_policy) : ControlNode::ControlNode(name)
{
    reset_policy_ = reset_policy;
    current_child_idx_ = 0;  // initialize the current running child
}
//USato affichè si crei una sola istanza del vettore per memorizzare lo stato dei nodi fallback
static bool vector_initialized = false;

BT::SequenceNodeWithMemory::~SequenceNodeWithMemory() {}

BT::ReturnStatus BT::SequenceNodeWithMemory::Tick()
{
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    // Routing the ticks according to the sequence node's (with memory) logic:
    N_of_children_ = children_nodes_.size();
    //DEBUG_STDOUT("valore N child "<< N_of_children_);
    //DEBUG_STDOUT("valore current_child index "<< current_child_idx_);

    if (!vector_initialized)
    {
        children_results_.resize(N_of_children_, false);
        vector_initialized = true;
    }

    while (current_child_idx_ < N_of_children_)
    {

        /*      Ticking an action is different from ticking a condition. An action executed some portion of code in another thread.
                We want this thread detached so we can cancel its execution (when the action no longer receive ticks).
                Hence we cannot just call the method Tick() from the action as doing so will block the execution of the tree.
                For this reason if a child of this node is an action, then we send the tick using the tick engine. Otherwise we call the method Tick() and wait for the response.
        */

        if (children_nodes_[current_child_idx_]->get_type() == BT::ACTION_NODE)
        {
            // 1) If the child i is an action, read its state.
            // Action nodes runs in another thread, hence you cannot retrieve the status just by executing it.

            child_i_status_ = children_nodes_[current_child_idx_]->get_status();
            DEBUG_STDOUT(get_name() << " It is an action " << children_nodes_[current_child_idx_]->get_name()
                         << " with status: " << child_i_status_);

            if (child_i_status_ == BT::IDLE || child_i_status_ == BT::HALTED)
            {
                // 1.1) If the action status is not running, the sequence node sends a tick to it.
                DEBUG_STDOUT(get_name() << "NEEDS TO TICK " << children_nodes_[current_child_idx_]->get_name());
                //std::this_thread::sleep_for(std::chrono::seconds(2));
                children_nodes_[current_child_idx_]->tick_engine.Tick();

                // waits for the tick to arrive to the child
                do
                {
                    child_i_status_ = children_nodes_[current_child_idx_]->get_status();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                while (child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS
                       && child_i_status_ != BT::FAILURE);
            }
        }
        else
        {
            // 2) if it's not an action:
            // Send the tick and wait for the response;
            child_i_status_ = children_nodes_[current_child_idx_]->Tick();
        }

        if (child_i_status_ == BT::SUCCESS ||child_i_status_ == BT::FAILURE )
        {
            // the child goes in idle if it has returned success or failure.
            children_results_[current_child_idx_] = children_nodes_[current_child_idx_]->get_result_value();

            DEBUG_STDOUT(" VALORE CHILDREN_RESULT SEQ: " << children_results_[current_child_idx_] << " !");
            //std::cout << "Contenuto del vettore:" << std::endl;
            //for (const int& element : children_results_) {
            //    std::cout << element << ' ';
            //}
            std::cout << std::endl;
             // the child goes in idle if it has returned success or failure.
            children_nodes_[current_child_idx_]->set_status(BT::IDLE);
        }

        if (child_i_status_ != BT::SUCCESS)
        {
            // If the  child status is not success, return the status
            //DEBUG_STDOUT("the status of: " << get_name() << " becomes " << child_i_status_);
            if (child_i_status_ == BT::FAILURE && (reset_policy_ == BT::ON_FAILURE
                                                   || reset_policy_ == BT::ON_SUCCESS_OR_FAILURE))
            {
                current_child_idx_ = 0;
            }
            set_status(child_i_status_);
            return child_i_status_;
        }
        else if (current_child_idx_ != N_of_children_ - 1)
        {
            // If the  child status is success, continue to the next child
            // (if any, hence if(current_child_ != N_of_children_ - 1) ) in the for loop (if any).
            current_child_idx_++;
        }
        else
        {
            // if it the last child.
            if (child_i_status_ == BT::SUCCESS)
            {
                ros::NodeHandle n;
                ros::Publisher failed_nodes_pub = n.advertise<std_msgs::String>("/topic_result_bt", 1000);
                ros::Rate loop_rate(10);
                int count = 0;
                bool should_continue = true;

                // Utilizza la funzione per inviare i nodi falliti
                std::stringstream failed_nodes_str;
                failed_nodes_str << "Non sono riuscito a raggiungere le destinazioni: ";
                bool has_failed_nodes = false;
                        
                for (size_t i = 0; i < children_results_.size(); ++i) {
                    DEBUG_STDOUT("VALUE CHILDREN RESULTS ::::::::::::::  " << children_results_[i] << " !");
                    if (!children_results_[i]) {
                        has_failed_nodes = true;
                        failed_nodes_str << " " <<  i+1 << ", ";
                    }
                }                
                while (ros::ok() and should_continue) {

                        // Controlla se dovresti uscire dal ciclo
                    if (count == 2) {
                        should_continue = false;
                    }
                    
                    // Utilizza la funzione per inviare i nodi falliti

                        std::string str = failed_nodes_str.str(); // Convert std::stringstream to std::string
                        DEBUG_STDOUT( "has_failed_nodes" << has_failed_nodes << " !");
                        if (has_failed_nodes) {
                            if (str.length() >= 2) {
                                str = str.substr(0, str.length() - 2); // Use substr on the std::string
                            }

                            std_msgs::String failed_nodes_msg;
                            failed_nodes_msg.data = str + ". Ritorno al menu principale!";
                            failed_nodes_pub.publish(failed_nodes_msg);
                        }else{
                            std_msgs::String success_nodes_msg;
                            success_nodes_msg.data = "Routine eseguita con successo! Ritorno al menù principale";
                            failed_nodes_pub.publish(success_nodes_msg);
                        }

                    ros::spinOnce();
                    loop_rate.sleep();
                    ++count;
                }

                //DEBUG_STDOUT("Routine terminata con successo! E messaggio inviato a RASA");
                
                // if it the last child and it has returned SUCCESS, reset the memory
                current_child_idx_ = 0;
            }
        
            set_status(child_i_status_);
            return child_i_status_;
        }
    }
    return BT::EXIT;
}


int BT::SequenceNodeWithMemory::DrawType()
{
    return BT::SEQUENCESTAR;
}


void BT::SequenceNodeWithMemory::Halt()
{
    current_child_idx_ = 0;
    BT::ControlNode::Halt();
}
