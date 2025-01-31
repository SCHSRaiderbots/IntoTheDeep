package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tray {
    Servo servo;

    public Tray(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "Tray");

        servo.setPosition(0.5);
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);

    }
}

