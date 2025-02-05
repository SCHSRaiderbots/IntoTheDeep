package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;

public class TrayDump extends Command {
    private final Tray m_tray;
    private final ElapsedTime elapsed = new ElapsedTime();

    public TrayDump(Tray tray) {
        // remember the subsystem
        m_tray = tray;
    }

    @Override
    public void initialize() {
        // start the timer
        elapsed.reset();

        // move the tray
        m_tray.setPosition(1.0);
    }

    @Override
    public boolean isFinished() {
        return elapsed.seconds() > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        // return the tray to the default position
        m_tray.setPosition(0.5);
    }
}
