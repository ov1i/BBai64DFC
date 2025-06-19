#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

class PWMController {
public:
    void init();
    void updateDuty(int duty_ns);
};

#endif
