#pragma once

enum class ClawAction {
    OPEN,
    CLOSE
};

void claw_control(ClawAction action);
void claw_idle();