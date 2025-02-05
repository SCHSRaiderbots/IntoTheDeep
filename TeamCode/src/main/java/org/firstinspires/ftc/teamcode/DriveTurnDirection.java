package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class DriveTurnDirection extends Command {
    final double m_x;
    final double m_y;

    public DriveTurnDirection(double x, double y) {
        // remember the direction
        m_x = x;
        m_y = y;
    }

    @Override
    public void initialize() {
        // TODO: This is wrong...
        Motion.headingInches(m_x, m_y);
    }

    @Override
    public boolean isFinished() {
        return Motion.finished();
    }
}