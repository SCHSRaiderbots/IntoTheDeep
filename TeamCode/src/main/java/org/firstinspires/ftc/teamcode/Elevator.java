package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    enum ElevatorPosition {
        BOTTOM(0.0),
        LOW_NET(25.5),
        HIGH_NET(42.75);

        final int m_height;

        ElevatorPosition(double height) {
            m_height = (int)height;
        }
    }
    /** motor that drives the elevator */
    DcMotorEx motor;

    public Elevator(HardwareMap hardwareMap) {
        // get the motor
        motor = hardwareMap.get(DcMotorEx.class, "motorElevator");

        motor.setTargetPosition(0);

        // use PID control
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1.0);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void setTargetPosition(int pos) {
        motor.setTargetPosition(pos);
    }

    public void setTargetPosition(ElevatorPosition pos) {
        setTargetPosition(pos.m_height);
    }

    public void setTargetPositionInches(double inches) {
        double ticks = (inches - 0.75) * (1900.0 / 37.5);

        setTargetPosition((int)ticks);
    }

    /** position in ticks */
    public void setPosition(double position) {
        int ticks = (int)position;

        motor.setTargetPosition(ticks);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
