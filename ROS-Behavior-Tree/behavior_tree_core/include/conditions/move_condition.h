#ifndef CONDITIONS_CONDITION_TEST_NODE_H
#define CONDITIONS_CONDITION_TEST_NODE_H
#endif  // CONDITIONS_CONDITION_TEST_NODE_H
#include <condition_node.h>
#include <string>

namespace BT
{
    class ConditionMove : public ConditionNode
    {
    public:
        // Constructor
        explicit ConditionMove(std::string Name, int id);
        ~ConditionMove();
        void set_boolean_value(bool boolean_value);

        BT::ReturnStatus Tick();
    private:
        bool boolean_value_;
        int id_;
    };
}  // namespace BT