package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Wrist {
    enum WristPosition {
        GROUND(-41.1),
        SKIM(-36.0),
        HORIZ(0.0),
        BASKET(135.0);

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
        setPosition(WristPosition.GROUND);

        // set the mode
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the power
        motor.setPower(0.8);
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
    }

    public void setPosition(WristPosition pos) {
        // simply move to the enumerated position
        setPosition(pos.angle);
    }
}
