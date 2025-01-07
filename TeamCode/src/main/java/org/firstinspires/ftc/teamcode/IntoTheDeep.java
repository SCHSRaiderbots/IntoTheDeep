package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Static class that contains game information.
 * The game information should survive the autonomous to driver controlled transition.
 * <p>
 *     That is, if we initialize the Alliance and starting position during autonomous,
 *     that information should be present during teleop.
 * </p>
 * <p>
 *     information: alliance, startPosition, lane?
 * </p>
 */
public class IntoTheDeep {
    /** We can be on the BLUE or the RED alliance */
    public enum Alliance {BLUE, RED}
    /** Our current alliance */
    public static Alliance alliance = Alliance.RED;

    /** We can start in the NET_ZONE or the OBSERVATION_ZONE */
    public enum StartPos {NET_ZONE, OBSERVATION_ZONE}
    /** Starting Position of the Robot */
    public static StartPos startPos = StartPos.OBSERVATION_ZONE;

    // use the authoritative distance in Motion.
    public static final float inchTile = (float)Motion.inchesPerTile;

    // several routines that will convert tiles to inches
    // TODO: Move to Motion!
    public static double tileX(double tiles) { return tiles * Motion.inchesPerTile; }
    public static double tileX(double tiles, double inches) { return tiles * Motion.inchesPerTile + inches; }
    public static double tileY(double tiles) { return tiles * Motion.inchesPerTile; }
    public static double tileY(double tiles, double inches) { return tiles * Motion.inchesPerTile + inches; }

    @SuppressWarnings("unused")
    public static double tileXR(double tiles) { return tileX(tiles); }
    @SuppressWarnings("unused")
    public static double tileXR(double tiles, double inches) { return tileX(tiles, inches); }
    @SuppressWarnings("unused")
    public static double tileYR(double tiles) {
        switch (alliance) {
            case BLUE:
                return -tileY(tiles);
            case RED:
            default:
                return tileY(tiles);
        }
    }
    @SuppressWarnings("unused")
    public static double tileYR(double tiles, double inches) {
        switch (alliance) {
            case BLUE:
                return -tileY(tiles, inches);
            case RED:
            default:
                return tileY(tiles, inches);
        }
    }

    public static void init() {
        // set the robot's initial pose
        setPose();
    }

    /**
     * Called during the Autonomous init_loop.
     * Obtains configuration information from the gamepad.
     * @param gamepad1 Driver's gamepad
     */
    public static void init_loop(Telemetry telemetry, Gamepad gamepad1) {
        // set the alliance
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
            // update the initial pose
            setPose();
        }
        if (gamepad1.b) {
            alliance = Alliance.RED;
            // update the initial pose
            setPose();
        }

        // set the starting position
        if (gamepad1.dpad_up) {
            startPos = StartPos.OBSERVATION_ZONE;
            // update the initial pose
            setPose();
        }
        if (gamepad1.dpad_down) {
            startPos = StartPos.NET_ZONE;
            // update the initial pose
            setPose();
        }

        // possibly change starting conditions
        telemetry.addData("alliance: x (Blue) or b (Red)", alliance);
        telemetry.addData("startPos: dpad up or down", startPos);
    }


    /**
     * Set the robot pose based on the alliance and the starting position.
     */
    public static void setPose() {
        // TODO: robotBackDistance should be a property of the robot
        double robotBackDistance = 9.25;

        /// set the starting x
        // TODO: assumes alliance wall is 70.75 (x2 = 141.5)
        // the wall is at 3 tiles less 3/8 inch (1/2 the interface gap)
        double x = (startPos == StartPos.NET_ZONE) ? tileX(-1.0) : tileX(+1.0);

        // could just do an asymmetric flip
        if (alliance == Alliance.BLUE) {
            // flip x
            x = -x;
        }

        // set the starting y -- only depends on Alliance
        double y = (alliance == Alliance.RED) ? tileY(-3, robotBackDistance) : tileY(+3, -robotBackDistance);

        // the angle depends on just the alliance
        double ang = (alliance == Alliance.RED)? +90.0 : -90.0;

        // set the new pose
        Motion.setPoseInches(x, y, ang);
    }

}