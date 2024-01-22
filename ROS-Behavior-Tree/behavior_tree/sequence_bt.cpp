#include <actions/move_to_position.h>
#include <string>

BT::MoveToPosition::MoveToPosition(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    boolean_value_ = true;
    time_ = 3;
    thread_ = std::thread(&MoveToPosition::WaitForTick, this);
}

BT::MoveToPosition::~MoveToPosition() {}

void BT::MoveToPosition::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);
        // Perform action...
        int i = 0;
        while (get_status() != BT::HALTED && i++ < time_)
        {
            DEBUG_STDOUT(" Action " << get_name() << "running! Thread id: " << std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (get_status() != BT::HALTED)
        {
            if (boolean_value_)
            {
                set_status(BT::SUCCESS);
                DEBUG_STDOUT(" Action " << get_name() << " Done!");
            }
            else
            {
                set_status(BT::FAILURE);
                DEBUG_STDOUT(" Action " << get_name() << " FAILED!");
            }
        }
    }
}

void BT::MoveToPosition::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("HALTED state set!");
}

void BT::MoveToPosition::set_time(int time)
{
    time_ = time;
}

void BT::MoveToPosition::set_boolean_value(bool boolean_value)
{
    boolean_value_ = boolean_value;
}


int main(){
    
}