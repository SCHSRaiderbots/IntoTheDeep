// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.command;

import java.util.LinkedHashSet;
import java.util.Set;

public class CommandScheduler {
    /** the singleton instance of the CommandScheduler */
    private static CommandScheduler instance = null;

    // A set of the currently-running commands.
    private final Set<Command> m_scheduledCommands = new LinkedHashSet<>();

    // A map from required subsystems to their requiring commands. Also used as a set of the
    // currently-required subsystems.
    // private final Map<Subsystem, Command> m_requirements = new LinkedHashMap<>();

    // A map from subsystems registered with the scheduler to their default commands.  Also used
    // as a list of currently-registered subsystems.
    // private final Map<Subsystem, Command> m_subsystems = new LinkedHashMap<>();

    // private final EventLoop m_defaultButtonLoop = new EventLoop();
    // The set of currently-registered buttons that will be polled every iteration.
    // private EventLoop m_activeButtonLoop = m_defaultButtonLoop;

    private boolean m_disabled = false;

    /**
     * Make an instance of the scheduler
     */
    private CommandScheduler() {

    }

    /**
     * Obtain the singleton instance of the CommandScheduler
     * @return the scheduler
     */
    public static CommandScheduler getInstance() {
        // if the scheduler has not been made yet
        if (instance == null) {
            // then make it
            instance = new CommandScheduler();
        }

        // return the singleton
        return instance;
    }

    public void cancel(Command ... commands) {
        // TODO: implement
        for (Command command: commands) {
            m_scheduledCommands.remove(command);
        }
    }

    public void cancelAll() {
        // TODO: implement
        for (Command command: m_scheduledCommands) {
            // TODO: removing something in the list!
            // maybe end(true) all and then clear the list
            cancel(command);
        }
    }

    public void run() {
        // TODO implement
    }

    public void schedule(Command... commands) {
        // TODO implement
        for (Command command: commands) {
            m_scheduledCommands.add(command);
        }
    }

    /**
     * Whether the given commands are running. Note that this only works on commands that are directly
     * scheduled by the scheduler; it will not work on commands inside compositions, as the scheduler
     * does not see them.
     *
     * @param commands multiple commands to check
     * @return whether all of the commands are currently scheduled
     */
    public boolean isScheduled(Command... commands) {
        for (Command cmd : commands) {
            if (!isScheduled(cmd)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Whether the given command is running. Note that this only works on commands that are directly
     * scheduled by the scheduler; it will not work on commands inside compositions, as the scheduler
     * does not see them.
     *
     * @param command a single command to check
     * @return whether the command is currently scheduled
     */
    public boolean isScheduled(Command command) {
        return m_scheduledCommands.contains(command);
    }

    /** Disables the command scheduler. */
    public void disable() {
        m_disabled = true;
    }

    /** Enables the command scheduler. */
    public void enable() {
        m_disabled = false;
    }

}
