// Base class for state machine functions

#pragma once

#include "States/BaseState.h"
#include <AP_Common/AP_Common.h>
#include <map>

template<typename StateEnum, typename Context>
class StateMachine {
public:
    void change_state(StateEnum next, Context& ctx);
    void update(Context& ctx);

    void register_state(StateEnum id, BaseState* state);
private:
    std::map<StateEnum, BaseState*> _states;
    StateEnum _current;
};
