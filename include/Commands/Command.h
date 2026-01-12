#pragma once

#include <vector>
#include <functional>

enum class CommandCancelBehavior {

    CancelIncoming,

    CancelRunning,
};

class Command {
public:

    virtual void initialize() {}

    virtual void execute() {}

    virtual bool isFinished() { return false; };

    virtual void end(bool interrupted){};
     /*
    virtual std::vector<Subsystem *> getRequirements() { return {}; };

    virtual CommandCancelBehavior getCancelBehavior() {
        return CommandCancelBehavior::CancelRunning;
    };
   
    void schedule();

    void cancel();


    [[nodiscard]] bool scheduled() const;
    Command *andThen(Command *other);
    Command *withTimeout(QTime duration);
    Command *until(const std::function<bool()> &isFinish);
    Command *with(Command *other);
    Command *race(Command *other);
    Command *repeatedly();
    Command *asProxy();
    */
    virtual ~Command() = default;
};