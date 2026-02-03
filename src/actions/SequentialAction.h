#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
#include <vector>
struct SequentialAction : Action
{
    bool canceled = false;
    std::vector<Action> actions{};
    uint index = 0;
    SequentialAction(std::vector<Action> actions) : actions(actions) {}
    void cancel() override { canceled = true; }
    bool completed() const override { return canceled; }

    void run(MouseState &s, MouseIO &io) override
    {
        if (actions[index].completed())
        {
            index++;
        }
        if (index == actions.size())
        {
            canceled = true;
            return;
        }
        actions[index].run(s, io);
    }
};