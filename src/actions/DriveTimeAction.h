#include "Action.h"
#include "CommandGenerator.h"
#include "Commands.h"
struct DriveTimeAction : Action
{
    bool canceled = false;
    void cancel() override { canceled = true; }
    bool completed() const override { return canceled; }
    double totalTime = 0;
    double finalTime = 2;

    void run(MouseState &s, MouseIO &io) override
    {
        totalTime += io.getDt();
        if (totalTime > finalTime)
        {
            canceled = true;
            return;
        }
        io.drive(0.5, 0.5);
    }

    void setTime(double time)
    {
        finalTime = time;
    }
    void end(MouseState &s, MouseIO &io) override
    {
        io.drive(0.0, 0.0);
    }
};