// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.command;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class CommandScheduler {
    /** the singleton instance of the CommandScheduler */
    private static CommandScheduler instance = null;

    // A map from required subsystems to their requiring commands. Also used as a set of the
    // currently-required subsystems.
    private final Map<Subsystem, Command> m_requirements = new LinkedHashMap<>();

    // A map from subsystems registered with the scheduler to their default commands.  Also used
    // as a list of currently-registered subsystems.
    private final Map<Subsystem, Command> m_subsystems = new LinkedHashMap<>();

    // A set of the currently-running commands.
    private final Set<Command> m_scheduledCommands = new LinkedHashSet<>();

    private boolean m_disabled = false;

    // Flag and queues for avoiding ConcurrentModificationException if commands are
    // scheduled/canceled during run
    private boolean m_inRunLoop;
    private final Set<Command> m_toSchedule = new LinkedHashSet<>();
    private final List<Command> m_toCancel = new ArrayList<>();

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

    /** Cancels all commands that are currently scheduled. */
    public void cancelAll() {
        // Copy to array to avoid concurrent modification.
        cancel(m_scheduledCommands.toArray(new Command[0]));
    }

    /**
     * Runs a single iteration of the scheduler. The execution occurs in the following order:
     *
     * <p>Subsystem periodic methods are called.
     *
     * <p>Button bindings are polled, and new commands are scheduled from them.
     *
     * <p>Currently-scheduled commands are executed.
     *
     * <p>End conditions are checked on currently-scheduled commands, and commands that are finished
     * have their end methods called and are removed.
     *
     * <p>Any subsystems not being used as requirements have their default methods started.
     */
    public void run() {
        if (m_disabled) {
            return;
        }

        // Run the periodic method of all registered subsystems.
        for (Subsystem subsystem : m_subsystems.keySet()) {
            subsystem.periodic();
            /*
            if (RobotBase.isSimulation()) {
                subsystem.simulationPeriodic();
            }
             */
        }

        // Cache the active instance to avoid concurrency problems if setActiveLoop() is called from
        // inside the button bindings.
        // EventLoop loopCache = m_activeButtonLoop;
        // Poll buttons for new commands to add.
        // loopCache.poll();

        m_inRunLoop = true;
        // Run scheduled commands, remove finished commands.
        for (Iterator<Command> iterator = m_scheduledCommands.iterator(); iterator.hasNext(); ) {
            Command command = iterator.next();

            /*
            if (!command.runsWhenDisabled() && RobotState.isDisabled()) {
                command.end(true);
                for (Consumer<Command> action : m_interruptActions) {
                    action.accept(command);
                }
                m_requirements.keySet().removeAll(command.getRequirements());
                iterator.remove();
                continue;
            }
             */

            command.execute();
            /*
            for (Consumer<Command> action : m_executeActions) {
                action.accept(command);
            }
             */

            if (command.isFinished()) {
                command.end(false);
                /*
                for (Consumer<Command> action : m_finishActions) {
                    action.accept(command);
                }
                 */
                iterator.remove();
                m_requirements.keySet().removeAll(command.getRequirements());
            }
        }
        m_inRunLoop = false;

        // Schedule/cancel commands from queues populated during loop
        for (Command command : m_toSchedule) {
            schedule(command);
        }

        for (Command command : m_toCancel) {
            cancel(command);
        }

        m_toSchedule.clear();
        m_toCancel.clear();

        // Add default commands for un-required registered subsystems.
        for (Map.Entry<Subsystem, Command> subsystemCommand : m_subsystems.entrySet()) {
            if (!m_requirements.containsKey(subsystemCommand.getKey())
                    && subsystemCommand.getValue() != null) {
                schedule(subsystemCommand.getValue());
            }
        }
    }

    /**
     * Initializes a given command, adds its requirements to the list, and performs the init actions.
     *
     * @param command The command to initialize
     * @param requirements The command requirements
     */
    private void initCommand(Command command, Set<Subsystem> requirements) {
        m_scheduledCommands.add(command);
        for (Subsystem requirement : requirements) {
            m_requirements.put(requirement, command);
        }
        command.initialize();

        /*
        for (Consumer<Command> action : m_initActions) {
            action.accept(command);
        }

        m_watchdog.addEpoch(command.getName() + ".initialize()");
        */
    }

    public void schedule(Command command) {
        if (command == null) {
            return;
        }

        if (m_inRunLoop) {
            m_toSchedule.add(command);
            return;
        }

        // requireNotComposed(command);

        // Do nothing if the scheduler is disabled, the robot is disabled and the command doesn't
        // run when disabled, or the command is already scheduled.
        if (m_disabled
                || isScheduled(command)
                /* || RobotState.isDisabled() && !command.runsWhenDisabled() */) {
            return;
        }

        Set<Subsystem> requirements = command.getRequirements();

        /*
        // Schedule the command if the requirements are not currently in-use.
        if (Collections.disjoint(m_requirements.keySet(), requirements)) {
            initCommand(command, requirements);
        } else {
            // Else check if the requirements that are in use have all have interruptible commands,
            // and if so, interrupt those commands and schedule the new command.
            for (Subsystem requirement : requirements) {
                Command requiring = requiring(requirement);
                if (requiring != null
                        && requiring.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming) {
                    return;
                }
            }
            for (Subsystem requirement : requirements) {
                Command requiring = requiring(requirement);
                if (requiring != null) {
                    cancel(requiring);
                }
            }
            initCommand(command, requirements);
        }
         */
        initCommand(command, requirements);
    }

    public void schedule(Command... commands) {
        // TODO implement
        for (Command command: commands) {
            schedule(command);
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
