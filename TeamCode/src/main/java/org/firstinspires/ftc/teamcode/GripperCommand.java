package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class GripperCommand extends Command {
    final Gripper m_gripper;
    final double m_power;

    public GripperCommand(double power, Gripper gripper) {
        m_power = power;
        m_gripper = gripper;
    }

    public void initialize() {
        m_gripper.grip(m_power);
    }

    public boolean isFinished() {
        return true;
    }
}
