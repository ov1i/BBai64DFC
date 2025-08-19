#include "PWMController.h"
#include <ti/drv/pwm/PWM.h>
#include <ti/drv/pwm/PWM_v0.h>

PWM_Handle pwmHandle;

void PWMController::init() {
    PWM_Params params;
    PWM_Params_init(&params);

    params.periodValue = 20000000;   
    params.dutyValue   = 1500000;

    pwmHandle = PWM_open(0, &params);
    if (!pwmHandle) {
        // log error
    }

    PWM_start(pwmHandle);
}

void PWMController::updateDuty(int duty_ns) {
    if (pwmHandle) {
        PWM_setDuty(pwmHandle, duty_ns);
    }
}
