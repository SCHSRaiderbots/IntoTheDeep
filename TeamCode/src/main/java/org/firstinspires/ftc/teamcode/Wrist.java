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
        GROUND(-41.1),
        SKIM(-36.0),
        HORIZ(0.0),
        VERTICAL(90.0),
        BASKET(125.0),
        STOW(135.0);

        final double angle;

        WristPosition(double degrees) {
            // remember the desired angle
            angle = degrees;
        }
    }
    DcMotorEx motor;

    PIDFCoefficients pidfR2P = new PIDFCoefficients(1.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);

    public Wrist(HardwareMap hardwareMap) {
        // find the wrist's motor
        motor = hardwareMap.get(DcMotorEx.class, "motorWrist");

        // set motor direction so positive is up
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

        // set the initial target position (always start at ground level)
        // setPosition(WristPosition.GROUND);

        // set the mode
        // motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the power
        // motor.setPower(0.8);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPosition(double degrees) {
        // move the wrist to that angle

        double delta = degrees - WristPosition.GROUND.angle;

        // convert degrees to ticks
        int ticks = (int)((delta / 360.0) * 288.0 * (125.0 / 15.0));

        // set the target position
        motor.setTargetPosition(ticks);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the power so we do not drive into the ground
        setPower( (degrees < WristPosition.SKIM.angle) ? 0.2 : 1.0);
    }

    public void setPosition(WristPosition pos) {
        // simply move to the enumerated position
        setPosition(pos.angle);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void reset() {
        // set power level to 0
        motor.setPower(0.0);
        // reset the encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
