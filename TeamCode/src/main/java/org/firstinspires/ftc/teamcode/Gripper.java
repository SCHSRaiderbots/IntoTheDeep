package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    Servo servoGripper;

    private final double posFlat = 0.0;
    private final double posGrip = 0.5;

    public Gripper(HardwareMap hardwareMap) {
        // find the servo
        servoGripper = hardwareMap.get(Servo.class, "gripper");

        // set the servo to a known position
        servoGripper.setPosition(posFlat);
    }

    public void grip() {
        servoGripper.setPosition(posGrip);
    }

    public void release() {
        servoGripper.setPosition(posFlat);
    }
}
