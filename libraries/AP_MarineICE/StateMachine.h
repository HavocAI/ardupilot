// Base class for state machine functions

#pragma once

#include <map>
#include <cstdint>
#include "BaseState.h"

template<typename StateEnum, typename Context>
class StateMachine {
public:
    StateMachine() : _current_state_id(), _current_state(nullptr), _initialized(false) {}

    void register_state(StateEnum id, BaseState<Context>* state) {
        _states[id] = state;
    }

    void change_state(StateEnum next_state_id, Context& ctx) {
        if (!_initialized || next_state_id != _current_state_id) {
            if (_current_state) {
                _current_state->exit(ctx);
            }

            auto it = _states.find(next_state_id);
            if (it != _states.end()) {
                _current_state_id = next_state_id;
                _current_state = it->second;
                _current_state->enter(ctx);
                _initialized = true;
            }
        }
    }

    void update(Context& ctx) {
        if (_current_state) {
            _current_state->run(ctx);
        }
    }

    StateEnum current_state_id() const { return _current_state_id; }

private:
    std::map<StateEnum, BaseState<Context>*> _states;
    StateEnum _current_state_id;
    BaseState<Context>* _current_state;
    bool _initialized;
};
