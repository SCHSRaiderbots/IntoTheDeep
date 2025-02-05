package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class DriveTurnToward extends Command {
    /** x-coordinate (inches) */
    final double m_x;
    /** y-coordinate (inches) */
    final double m_y;

    /**
     * Turn the robot toward a specific position
     * @param x x-coordinate (inches)
     * @param y y-coordinate (inches)
     */
    public DriveTurnToward(double x, double y) {
        // remember the target position
        m_x = x;
        m_y = y;
    }

    @Override
    public void initialize() {
        Motion.headTowardInches(m_x, m_y);
    }

    @Override
    public boolean isFinished() {
        return Motion.finished();
    }
}