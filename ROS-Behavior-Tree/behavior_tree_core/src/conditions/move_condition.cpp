#include <conditions/move_condition.h>
#include <string>

BT::ConditionMove::ConditionMove(std::string name, int id) : ConditionNode::ConditionNode(name), id_(id)
{
    type_ = BT::CONDITION_NODE;
    boolean_value_ = true;
}

BT::ConditionMove::~ConditionMove() {}

BT::ReturnStatus BT::ConditionMove::Tick()
{
        if (get_status() == BT::EXIT)
        {
            // The behavior tree is going to be destroied
            return BT::EXIT;
        }

        // Condition checking and state update

        if (boolean_value_)
        {
            set_status(BT::SUCCESS);
            std::cout << get_name() << " returning Success" << BT::SUCCESS << "!" << std::endl;
            return BT::SUCCESS;
        }
        else
        {
            set_status(BT::FAILURE);
            std::cout << get_name() << " returning Failure" << BT::FAILURE << "!" << std::endl;
            return BT::FAILURE;
        }
}




void BT::ConditionMove::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}