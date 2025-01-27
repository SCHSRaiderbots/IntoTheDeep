package org.firstinspires.ftc.teamcode.command;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class CommandSchedulerTests {
    private static class Counter extends Command {
        int m_count;
        int m_counter;

        public Counter(int count) {
            m_count = count;
        }

        public void initialize() {
            m_counter = 0;
        }

        public void execute() {
            m_counter++;
        }

        public boolean isFinished() {
            return m_counter > m_count;
        }
    }

    @Test
    public void getInstanceTest() {
        // the scheduler should not be null
        assertNotNull(CommandScheduler.getInstance());

        CommandScheduler scheduler = CommandScheduler.getInstance();

        // make a command
        Command foo = new Counter(3);

        // that command should not be scheduled
        assertFalse(scheduler.isScheduled(foo));

        // schedule it
        foo.schedule();

        // now it should be scheduled
        assertTrue(scheduler.isScheduled(foo));

        // remove it
        foo.cancel();
        assertFalse(scheduler.isScheduled(foo));

        // use to get rid of warnings
        scheduler.disable();
        scheduler.enable();
    }

    @Test
    public void runSchedulerTest() {
        // get the scheduler
        CommandScheduler scheduler = CommandScheduler.getInstance();

        // make a new command.
        Command count5 = new Counter(5);

        Command count3 = new Counter(3);

        // schedule it (runs initialize())
        count5.schedule();

        // make sure it is scheduled
        assertTrue(scheduler.isScheduled(count5));

        // count 3 has not been scheduled
        assertFalse(scheduler.isScheduled(count3));
        // all of these are not scheduled
        assertFalse(scheduler.isScheduled(count3, count5));

        // not finished
        assertFalse(count5.isFinished());

        // run twice
        scheduler.run();
        scheduler.run();

        // still false
        assertFalse(count5.isFinished());
        assertTrue(scheduler.isScheduled(count5));

        // run 4 more times
        scheduler.run();
        scheduler.run();
        scheduler.run();
        scheduler.run();

        // command should have finished
        assertTrue(count5.isFinished());
        assertFalse(scheduler.isScheduled(count5));


        // Run it again...

        // schedule it (runs initialize())
        count5.schedule();

        // make sure it is scheduled
        assertTrue(scheduler.isScheduled(count5));

        // not finished
        assertFalse(count5.isFinished());

        // run twice
        scheduler.run();
        scheduler.run();

        // still false
        assertFalse(count5.isFinished());
        assertTrue(scheduler.isScheduled(count5));

        // run 4 more times
        scheduler.run();
        scheduler.run();
        scheduler.run();
        scheduler.run();

        // command should have finished
        assertTrue(count5.isFinished());
        assertFalse(scheduler.isScheduled(count5));
    }

    @Test
    public void cancelAllTest() {
        CommandScheduler scheduler = CommandScheduler.getInstance();

        Command count3 = new Counter(3);
        Command count5 = new Counter(5);

        count3.schedule();
        count5.schedule();

        assertTrue(scheduler.isScheduled(count3));
        assertTrue(scheduler.isScheduled(count5));

        scheduler.cancelAll();

        assertFalse(scheduler.isScheduled(count3));
        assertFalse(scheduler.isScheduled(count5));
    }
}
