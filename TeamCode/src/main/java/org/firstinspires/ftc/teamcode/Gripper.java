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
        // start gripping
        servoGripper.setPosition(posGrip);
    }

    public void grip(boolean bGrip) {
        servoGripper.setPosition((bGrip)? posGrip : posFlat);
    }

    public void release() {
        // release the grip
        servoGripper.setPosition(posFlat);
    }
}
