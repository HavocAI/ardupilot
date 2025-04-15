#pragma once

#include "BaseState.h"

class AP_MarineICE;

class State_Trim_Manual : public BaseState<AP_MarineICE> {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};

class State_Trim_Auto_Stop : public BaseState<AP_MarineICE> {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};

class State_Trim_Auto_Up : public BaseState<AP_MarineICE> {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};

class State_Trim_Auto_Down : public BaseState<AP_MarineICE> {
public:
    void enter(AP_MarineICE &marine_ice) override;
    void run(AP_MarineICE &marine_ice) override;
    void exit(AP_MarineICE &marine_ice) override;
};
