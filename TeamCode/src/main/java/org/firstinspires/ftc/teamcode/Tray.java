package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tray {
    private final Servo servo;

    public Tray(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Tray");

        servo.setPosition(0.5);

        LogDevice.dump("tray", servo);
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);

    }
}

