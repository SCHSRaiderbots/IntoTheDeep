// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.command;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/**
 * Make a command-based system similar to the FRC WPILib.
 * <p>
 *     WPILib make Command an interface... The interface has the decorations.
 * </p>
 */
public abstract class Command {
    /** Requirements set. */
    private final Set<Subsystem> m_requirements = new HashSet<>();

    // getName()
    // setName()
    // getSubsystem()
    // setSubsystem()

    /**
     * Initialize a command.
     * This method sets everything up for the command to start running.
     */
    public void initialize() {
        // initialize the command
    }

    /**
     * Make some progress on completing the command.
     */
    public void execute() {
        // execute the command
    }

    /**
     * Test if the command has finished running.
     * @return true if the command is done.
     */
    public boolean isFinished() {
        // test if the command is finished
        return false;
    }

    /**
     * Clean up after the command has finished (or been interrupted).
     * @param interrupted true if the command is being interrupted.
     */
    public void end(boolean interrupted) {
        // clean up the command
    }

    /**
     * Specifies the set of subsystems used by this command. Two commands cannot use the same
     * subsystem at the same time. If another command is scheduled that shares a requirement, {@link
     * #getInterruptionBehavior()} will be checked and followed. If no subsystems are required, return
     * an empty set.
     *
     * <p>Note: it is recommended that user implementations contain the requirements as a field, and
     * return that field here, rather than allocating a new set every time this is called.
     *
     * @return the set of subsystems that are required
     * @see InterruptionBehavior
     */
    public Set<Subsystem> getRequirements() {
        return m_requirements;
    }

    /**
     * Adds the specified subsystems to the requirements of the command. The scheduler will prevent
     * two commands that require the same subsystem from being scheduled simultaneously.
     *
     * <p>Note that the scheduler determines the requirements of a command when it is scheduled, so
     * this method should normally be called from the command's constructor.
     *
     * @param requirements the requirements to add
     */
    public final void addRequirements(Subsystem... requirements) {
        for (Subsystem requirement : requirements) {
            // m_requirements.add(requireNonNullParam(requirement, "requirement", "addRequirements"));
            if (requirement != null) {
                m_requirements.add(requirement);
            }
        }
    }

    /**
     * Adds the specified subsystems to the requirements of the command. The scheduler will prevent
     * two commands that require the same subsystem from being scheduled simultaneously.
     *
     * <p>Note that the scheduler determines the requirements of a command when it is scheduled, so
     * this method should normally be called from the command's constructor.
     *
     * @param requirements the requirements to add
     */
    public final void addRequirements(Collection<Subsystem> requirements) {
        m_requirements.addAll(requirements);
    }

    /** Schedules this command. */
    public void schedule() {
        CommandScheduler.getInstance().schedule(this);
    }

    /**
     * Cancels this command. Will call {@link #end(boolean) end(true)}. Commands will be canceled
     * regardless of {@link InterruptionBehavior interruption behavior}.
     *
     * @see CommandScheduler#cancel(Command...)
     */
    public void cancel() {
        CommandScheduler.getInstance().cancel(this);
    }

    /**
     * Whether the command is currently scheduled. Note that this does not detect whether the command
     * is in a composition, only whether it is directly being run by the scheduler.
     *
     * @return Whether the command is scheduled.
     */
    public boolean isScheduled() {
        return CommandScheduler.getInstance().isScheduled(this);
    }

    /**
     * Whether the command requires a given subsystem.
     *
     * @param requirement the subsystem to inquire about
     * @return whether the subsystem is required
     */
    public boolean hasRequirement(Subsystem requirement) {
        return getRequirements().contains(requirement);
    }

    /**
     * How the command behaves when another command with a shared requirement is scheduled.
     *
     * @return a variant of {@link InterruptionBehavior}, defaulting to {@link
     *     InterruptionBehavior#kCancelSelf kCancelSelf}.
     */
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

    /**
     * Whether the given command should run when the robot is disabled. Override to return true if the
     * command should run when disabled.
     *
     * @return whether the command should run when the robot is disabled
     */
    public boolean runsWhenDisabled() {
        return false;
    }

    /**
     * An enum describing the command's behavior when another command with a shared requirement is
     * scheduled.
     */
    public enum InterruptionBehavior {
        /**
         * This command ends, {@link #end(boolean) end(true)} is called, and the incoming command is
         * scheduled normally.
         *
         * <p>This is the default behavior.
         */
        kCancelSelf,
        /** This command continues, and the incoming command is not scheduled. */
        kCancelIncoming
    }
}