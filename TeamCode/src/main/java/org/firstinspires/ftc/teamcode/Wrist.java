package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist {
    enum WristPosition {
        GROUND(-20.0),
        HORIZ(0.0),
        BASKET(135.0);

        final double angle;

        WristPosition(double degrees) {
            angle = degrees;
        }
    }
    DcMotorEx motor;

    public Wrist(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motorWrist");
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPosition(double degrees) {
        // move the wrist to that angle
    }

    public void setPosition(WristPosition pos) {
        setPosition(pos.angle);
    }
}
