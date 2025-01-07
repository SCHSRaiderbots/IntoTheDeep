package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/**
 * Some simple tests.
 * One of the goals is to remove some unused warnings.
 */
public class MotionTests {

    @Test
    public void constantTests() {
        final double eps = 0.0001;
        assertEquals(5, 1 + 4);

        assertEquals(Motion.CORE_HEX_TICKS_PER_REV, 4 * 72.0, eps);
        assertEquals(Motion.HD_HEX_TICKS_PER_REV, (56.0 / 2), eps);

        // VersaPlanetary Gear Ratios
        // Better as (ring + sun) / (sun) = (55 + 29) / 29
        assertEquals(Motion.HD_HEX_GEAR_CART_3_1, 84.0/29.0, eps);
        assertEquals(Motion.HD_HEX_GEAR_CART_4_1, 76.0/21.0, eps);
        assertEquals(Motion.HD_HEX_GEAR_CART_5_1, 68.0/13.0, eps);
    }
}
