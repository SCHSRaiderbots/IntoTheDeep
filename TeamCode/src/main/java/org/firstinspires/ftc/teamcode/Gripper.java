package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    CRServo servoGripper;

    public Gripper(HardwareMap hardwareMap) {
        // find the servo
        servoGripper = hardwareMap.get(CRServo.class, "gripper");

        // set the servo to a known value
        grip(0.0);
    }

    public void grip(double speed) {
        // start gripping
        servoGripper.setPower(speed);
    }
}
