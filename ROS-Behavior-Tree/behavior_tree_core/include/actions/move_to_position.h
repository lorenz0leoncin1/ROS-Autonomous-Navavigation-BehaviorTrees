#ifndef ACTIONS_MOVE_TO_POSITION1_H
#define ACTIONS_MOVE_TO_POSITION1_H

#include <action_node.h>
#include <string>

namespace BT
{
    class MoveToPosition1 : public ActionNode
    {
    public:
        // Costruttore
        explicit MoveToPosition1(std::string Name, int id);
        ~MoveToPosition1();

        void WaitForTick();
        void Halt();
        void set_time(int time);
        bool get_boolean_value();
        int  get_id_value();
        bool get_result_value();
        
    private:
        void set_boolean_value(bool boolean_value);
        int time_;
        bool boolean_value_;
        int id_;
        bool result_value_;
    };

    class FaultNodes : public ActionNode
    {
    public:
        // Costruttore
        explicit FaultNodes(std::string Name, int id);
        ~FaultNodes();

        void WaitForTick();
        void set_time(int time);

        void Halt();
        bool get_boolean_value();
        bool get_result_value();
        
    private:
        void set_boolean_value(bool boolean_value);
        int time_;
        bool boolean_value_;
        int id_;
        bool result_value_;
    };

} // namespace BT

#endif // ACTIONS_MOVE_TO_POSITION1_H