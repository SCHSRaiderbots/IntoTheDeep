package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class DriveTo extends Command {
    double m_x;
    double m_y;

    public DriveTo(double x, double y) {
        // remember the goal position
        m_x = x;
        m_y = y;
    }

    @Override
    public void initialize() {
        double d = Motion.distanceToInches(m_x, m_y);

        Motion.moveInches(d);
    }

    public boolean isFinished() {
        return Motion.finished();
    }
}