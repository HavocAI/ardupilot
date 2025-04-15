#pragma once

#include "BaseState.h"

class State_Trim_Manual : public BaseState {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};

class State_Trim_Auto_Stop : public BaseState {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};

class State_Trim_Auto_Up : public BaseState {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};

class State_Trim_Auto_Down : public BaseState {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};
