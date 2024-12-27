package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist {
    DcMotorEx motor;

    public Wrist(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motorWrist");
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
