package org.firstinspires.ftc.teamcode.command;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class CommandTest {
    private class Foo extends Command {

    }
    private class Counter extends Command {
        int m_countMax;
        int m_count;

        Counter(int countMax) {
            m_countMax = countMax;
        }

        @Override
        public void initialize() {
            m_count = 0;
        }

        @Override
        public void execute() {
            m_count++;
        }

        @Override
        public boolean isFinished() {
            return m_count >= m_countMax;
        }
    }

    @Test
    public void simpleTest() {
        Command foo = new Foo();

        // foo has no requirements
        assertTrue(foo.getRequirements().isEmpty());

        // does not run when disabled
        assertFalse(foo.runsWhenDisabled());

        // has default interruption behavior
        assertTrue(foo.getInterruptionBehavior() == Command.InterruptionBehavior.kCancelSelf);

        // initialize the command
        foo.initialize();
        // execute the command
        foo.execute();
        // check that it is not finished (default behavior)
        assertFalse(foo.isFinished());
        // end the command
        foo.end(true);
    }

    @Test
    public void terminationTest() {
        Command count5 = new Counter(5);

        count5.initialize();
        assertFalse(count5.isFinished());
        count5.execute();
        count5.execute();
        count5.execute();
        count5.execute();
        assertFalse(count5.isFinished());
        count5.execute();
        assertTrue(count5.isFinished());

        // cleanly end the command
        count5.end(false);
    }

    @Test
    public void sequentialTest() {
        Command count3 = new Counter(3);
        Command count5 = new Counter(5);
        Command seq = new SequentialCommandGroup(count3, count5);

        seq.initialize();
        assertFalse(seq.isFinished());
        seq.execute();
        seq.execute();
        seq.execute();
        assertFalse(seq.isFinished());
        seq.execute();
        seq.execute();
        seq.execute();
        seq.execute();
        seq.execute();
        assertTrue(seq.isFinished());

        // cleanly end the command
        seq.end(false);
    }

    @Test
    public void parallelTest() {
        Command count3 = new Counter(3);
        Command count5 = new Counter(5);
        Command para = new ParallelCommandGroup(count3, count5);

        para.initialize();
        assertFalse(para.isFinished());
        para.execute();
        para.execute();
        para.execute();
        assertFalse(para.isFinished());
        para.execute();
        para.execute();
        assertTrue(para.isFinished());

        // cleanly end the command
        para.end(false);
    }
}
