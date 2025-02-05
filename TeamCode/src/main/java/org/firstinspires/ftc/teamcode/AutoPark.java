package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Wrist.WristPosition;
import static org.firstinspires.ftc.teamcode.Elevator.ElevatorPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.SequentialCommandGroup;

@Autonomous(name="Auto Park", group="testing")
@SuppressWarnings("unused")
public class AutoPark extends OpMode {
    Command command;
    Vision vision;
    // Arm arm;
    Wrist wrist;
    Tray tray;
    Gripper gripper;
    Elevator elevator;

    @Override
    public void init() {
        // identify the robot
        Motion.robot = RobotId.identifyRobot(hardwareMap);
        Motion.init(hardwareMap);

        // get the vision system
        vision = new Vision(hardwareMap);

        // get the wrist
        wrist = new Wrist(hardwareMap);
        // reset the wrist encoder in auto
        wrist.reset();
        // go to the stow position
        wrist.setPosition(WristPosition.STOW);

        // get the gripper
        gripper = new Gripper(hardwareMap);

        // get the tray
        tray = new Tray(hardwareMap);

        // get the elevator
        elevator = new Elevator(hardwareMap);
        // reset the encoder
        elevator.reset();
        // set the target position
        elevator.setTargetPosition(ElevatorPosition.BOTTOM);

        IntoTheDeep.init();

        // set the starting pose
        Motion.setPoseInches(-44, -60, 135.0);
        Motion.setPower(0.4);

        // make the command (so end() will not crash with a null pointer)
        command = new SequentialCommandGroup(

                // back into the net
                new DriveForward(4.0),
                new DriveTurnToward(72.0, -48.0),
                new DriveForward(36.0));

    }

    @Override
    public void init_loop() {
        // figure our position
        Motion.updateRobotPose();

        // collect the starting information
        IntoTheDeep.init_loop(telemetry, gamepad1);

        if (wrist.isFinished()) {
            wrist.setPower(0.0);
        }

        // report the starting position
        telemetry.addData("pose", "%8.2f %8.2f %8.2f", Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);
    }

    @Override
    public void start() {
        // set the starting pose
        Motion.setPoseInches(24.0, -60, 90.0);
        Motion.setPower(0.4);

        command.initialize();
        command.execute();
    }

    @Override
    public void loop() {
        // figure our position
        Motion.updateRobotPose();


        // report the starting position
        telemetry.addData("pose", "%8.2f %8.2f %8.2f", Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);


        if (!command.isFinished()) {
            command.execute();
        }
    }

    @Override
    public void stop() {
        // TODO: make a decent scheduler
        command.end(true);
    }
}