package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public enum ElevatorPosition {
        BOTTOM(0.75),
        LOW_NET(25.5),
        HIGH_NET(35.0 + 0.0 * 42.75);

        final double m_height;

        ElevatorPosition(double height) {
            m_height = height;
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

        setPower(1.0);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    // set the elevator position in ticks
    public void setTargetPosition(int pos) {

        motor.setTargetPosition(pos);
    }

    public void setTargetPosition(ElevatorPosition pos) {
        setTargetPosition(pos.m_height);
    }

    public void setTargetPosition(double inches) {
        double ticks = (inches - 0.75) * (1900.0 / 37.5);

        setTargetPosition((int)ticks);
    }

    public boolean isFinished() {
        return !motor.isBusy();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
