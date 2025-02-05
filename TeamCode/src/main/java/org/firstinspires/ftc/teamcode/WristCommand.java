package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class WristCommand extends Command {
    Wrist m_wrist;
    Wrist.WristPosition m_wristPos;

    public WristCommand(Wrist.WristPosition pos, Wrist wrist) {
        m_wrist = wrist;
        m_wristPos = pos;
    }

    public void initialize() {
        m_wrist.setPosition(m_wristPos);
    }

    public boolean isFinished() {
        return m_wrist.isFinished();
    }
}
