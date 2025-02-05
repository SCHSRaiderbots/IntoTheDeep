package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;

public class Delay extends Command {
    /** timer */
    final ElapsedTime elapsed = new ElapsedTime();

    /** seconds of delay */
    final double m_delay;

    public Delay(double seconds) {
        // remember the delay time
        m_delay = seconds;
    }

    @Override
    public void initialize() {
        elapsed.reset();
    }

    @Override
    public boolean isFinished() {
        return elapsed.seconds() > m_delay;
    }
}