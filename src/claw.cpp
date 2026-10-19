#include "claw.h"
#include "globals.h"
#include <cmath>

namespace {
bool torque_limit_reached = false;
constexpr double torque_limit_nm = 0.25;

bool update_torque_limit() {
    if (!torque_limit_reached && std::abs(clawMotor.get_torque()) >= torque_limit_nm) {
        torque_limit_reached = true;
    }
    return torque_limit_reached;
}
}

void claw_control(ClawAction action) {
    if (action == ClawAction::OPEN) {
        torque_limit_reached = false;
        clawMotor.move_voltage(6000);
    } else {
        if (update_torque_limit()) {
            clawMotor.move_voltage(-6000);
        } else {
            clawMotor.move_voltage(-12000);
        }
    }
}

void claw_idle() {
    clawMotor.move_voltage(update_torque_limit() ? -6000 : 0);
}