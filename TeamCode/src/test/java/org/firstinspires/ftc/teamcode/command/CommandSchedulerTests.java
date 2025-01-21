package org.firstinspires.ftc.teamcode.command;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class CommandSchedulerTests {
    private static class Foo extends Command {

    }

    @Test
    public void getInstanceTest() {
        // the scheduler should not be null
        assertNotNull(CommandScheduler.getInstance());

        CommandScheduler scheduler = CommandScheduler.getInstance();

        // make a command
        Command foo = new Foo();

        // that command should not be scheduled
        assertFalse(scheduler.isScheduled(foo));

        // schedule it
        foo.schedule();

        // now it should be scheduled
        assertTrue(scheduler.isScheduled(foo));

        // remove it
        foo.cancel();
        assertFalse(scheduler.isScheduled(foo));
    }
}
