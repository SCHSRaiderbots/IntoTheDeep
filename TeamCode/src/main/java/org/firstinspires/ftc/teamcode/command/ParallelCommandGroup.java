package org.firstinspires.ftc.teamcode.command;

public class ParallelCommandGroup extends Command {
    Command[] aCommands;
    boolean[] bCommandDone;

    public ParallelCommandGroup(Command ... commands) {
        aCommands = commands;
        bCommandDone = new boolean[aCommands.length];

        for (int i = 0; i < aCommands.length; i++) {
            // assume the command is finished
            bCommandDone[i] = true;
        }
    }

    @Override
    public void initialize() {
        // just start each of the commands
        for (int i = 0; i < aCommands.length; i++) {
            aCommands[i].initialize();
            bCommandDone[i] = false;
        }
    }

    @Override
    public void execute() {
        // make progress on each subcommand
        for (int i = 0; i < aCommands.length; i++) {
            // if the command has not finished, then execute it
            if (!bCommandDone[i]) {
                aCommands[i].execute();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // assume all commands have finished
        boolean done = true;

        for (int i = 0; i < aCommands.length; i++) {
            if (bCommandDone[i]) {
                // this command has already finished...
            } else if (aCommands[i].isFinished()) {
                // this command just finished
                aCommands[i].end(false);
                bCommandDone[i] = true;
            } else {
                // this command is still running...
                done = false;
            }
        }

        // return whether we are still alive
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < aCommands.length; i++) {
            if (!bCommandDone[i]) {
                aCommands[i].end(interrupted);
            }
        }
    }
}