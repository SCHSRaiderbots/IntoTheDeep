package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class ElevatorCommand extends Command {

    Elevator m_elevator;
    Elevator.ElevatorPosition m_pos;

    public ElevatorCommand(Elevator.ElevatorPosition pos, Elevator elevator) {
        m_pos = pos;
        m_elevator = elevator;
    }

    public void initialize() {
        m_elevator.setTargetPosition(m_pos);
    }

    public boolean isFinished() {
        return m_elevator.isFinished();
    }

}
