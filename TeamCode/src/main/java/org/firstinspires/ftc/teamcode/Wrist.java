package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public enum WristPosition {
        /** shovel touches the ground */
        GROUND(-41.1),
        /** shovel is just above the ground */
        SKIM(-36.0),
        /** shovel is horizontal so it can get into the submersible */
        HORIZ(0.0),
        /** shovel is vertical (out of the way of the elevator and tray */
        VERTICAL(90.0),
        /** shovel is ready to transfer to the basket */
        BASKET(125.0),
        /** shovel is inside the 18-inch cube */
        STOW(135.0);

        /** target angle of the wrist */
        final double angle;

        WristPosition(double degrees) {
            // remember the desired target angle
            angle = degrees;
        }
    }

    /** the motor that moves the wrist */
    private DcMotorEx motor;
    /** Gear ratio for 15-tooth motor pinion driving 125-tooth wrist gear */
    private static final double gearRatio = 125.0 / 15.0;

    /**
     * Construct an instance of the Wrist.
     * <p>Sets new PIDF coefficients for the motor control algorithm.
     * @param hardwareMap
     */
    public Wrist(HardwareMap hardwareMap) {
        // find the wrist's motor
        motor = hardwareMap.get(DcMotorEx.class, "motorWrist");

        // set motor direction so positive is up
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // calculate the PIDF coefficients
        // some useful methods
        // motor.getMotorType().getGearing();
        // motor.getMotorType().getAchieveableMaxTicksPerSecond();
        // motor.getMotorType().getAchieveableMaxRPMFraction();
        // motor.getMotorType().getMaxRPM();
        // motor.getMotorType().getTicksPerRev();
        //
        // this is the default value -- it has no F and significant I
        // PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 3.0, 0.0, 0.0, MotorControlAlgorithm.LegacyPID);
        // TODO: more intelligent values
        // I think that goes something like 32000 / maxTicksPerSecond
        // but where was that information?
        // https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/PIDFCoefficients.html
        // gives no information about the units.
        //
        // https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?tab=t.0#heading=h.61g9ixenznbx
        // says F should be 32767 / maxVelocity where max velocity was measured using motor.getVelocity()...
        // getVelocity() returns ticksPerSecond
        // which would be about 32000 / 558.96 = 57.25
        // double P = 10.0;
        // double F = 500.0;
        // an F of 500 seemed to give better performance....
        double F = 32767.0 / motor.getMotorType().getAchieveableMaxTicksPerSecond();
        final PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 0.0, 0.0, F, MotorControlAlgorithm.PIDF);

        // Run to Position PIDF
        // P was 1, but changed to 10 when F was changed above.
        final PIDFCoefficients pidfR2P = new PIDFCoefficients(10.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

        // describe the motor
        LogDevice.dump("wrist", motor);

        // The motor encoder is initialized by calling .reset() when the wrist is in GROUND position.
    }

    /**
     * Set the power for the wrist motor.
     * We reduce the power when the wrist is touching the ground.
     * @param power desired motor power (0 to 1.0)
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * Move the wrist to a particular target angle.
     * @param degrees desired wrist position in degrees from horizontal
     */
    public void setPosition(double degrees) {
        // calculate the desired encoder value in degrees
        double deltaDegrees = degrees - WristPosition.GROUND.angle;
        // convert to rotation
        double deltaRotations = deltaDegrees / 360.0;

        // convert encoder degrees to encoder ticks
        int ticks = (int)(deltaRotations * 288.0 * gearRatio);

        // set the target position
        motor.setTargetPosition(ticks);

        // tell the motor to go to that position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the power so we do not drive into the ground
        setPower( (degrees < WristPosition.SKIM.angle) ? 0.2 : 1.0);
    }

    /**
     * Set the wrist position to one of the enumerated positions.
     * @param pos desired position
     */
    public void setPosition(WristPosition pos) {
        // simply move to the enumerated position's angle
        setPosition(pos.angle);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    /**
     * Reset the wrist encoder.
     * Call this method when the wrist is touching the ground.
     */
    public void reset() {
        // set power level to 0 (the setPosition() method will set the power)
        motor.setPower(0.0);

        // reset the wrist encoder (mode will be reset in setPosition())
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
