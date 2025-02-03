package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Wrist.WristPosition;
import static org.firstinspires.ftc.teamcode.Elevator.ElevatorPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.command.SequentialCommandGroup;

@Autonomous(name="Auto Wave", group="testing")
@SuppressWarnings("unused")
public class AutoWave extends OpMode {
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
        // go to the basket position
        wrist.setPosition(WristPosition.BASKET);

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
                // new Delay(0.5),
                // drive forward
                // new DriveForward( tileX(2, 6.0))
                // new DriveTurnToward(tileXR(2.5), tileYR(-0.5)),
                // new DriveTo(tileX(2.5), tileYR(-0.5)),
                // new MoveArm(0, arm)

                new ParallelCommandGroup(
                    new WristCommand(WristPosition.HORIZ, wrist),
                    new SequentialCommandGroup(
                        new DriveForward(9.0),
                        new DriveTurnToward(0.0, 0.0)),
                    new ElevatorCommand(ElevatorPosition.HIGH_NET, elevator)),
                // align to score


                // raise the tray to score

                // back into the net
                new DriveForward(-6.0),
                new TrayDump(tray),


                new ParallelCommandGroup(
                        new DriveForward(6.0),
                        new ElevatorCommand(ElevatorPosition.TRANSFER, elevator),


                        new WristCommand(WristPosition.GROUND, wrist),

                        new GripperCommand(1.0,gripper)),



                // turn toward a prepositioned sample
                new DriveTurnToward(-46.5,-26.0),



                new DriveForward(15.0),
                new Delay(1.0),

                new ParallelCommandGroup(
                        new WristCommand(WristPosition.BASKET, wrist),
                        new DriveForward(-15.0)),


                // transfer sample to the tray
                new Delay(2.0),
                new GripperCommand(-0.35, gripper),
                new Delay(2.5),
                // drive back to the place to raise the elevator

                new WristCommand(WristPosition.VERTICAL, wrist),
                // align to basket
                new ParallelCommandGroup(

                        new DriveTurnToward(0,0),
                        new ElevatorCommand(ElevatorPosition.HIGH_NET, elevator)),

                new DriveForward(-6.0),
                new TrayDump(tray),
                new DriveForward(6.0),
                new ElevatorCommand(ElevatorPosition.BOTTOM,elevator)
                // new WristCommand(WristPosition.HORIZ, wrist),
                // new DriveForward (5.0),
                // new WristCommand(WristPosition.SKIM, wrist),
                // new GripperCommand(1.0, gripper),
                // new WristCommand(WristPosition.GROUND, wrist),
                // new DriveForward(6.0),
                // new DriveForward(2.0),
                // new DriveForward(2.0),

                // new WristCommand(WristPosition.HORIZ, wrist),
                // new GripperCommand(0.0, gripper),
                // new DriveForward(-3.0),
                // new WristCommand(WristPosition.BASKET, wrist),
                // new GripperCommand(1.0, gripper),
                // new WristCommand(WristPosition.VERTICAL, wrist),
                // new GripperCommand(0.0, gripper),

                // new WristCommand(WristPosition.GROUND, wrist),
                // new DriveTurnToward(0,0),
                // new DriveForward(-29.0),
                // new ElevatorCommand(ElevatorPosition.HIGH_NET, elevator),
                // new TrayDump(tray),
                // new ElevatorCommand(ElevatorPosition.BOTTOM, elevator)
        );
    }

    @Override
    public void init_loop() {
        // figure our position
        Motion.updateRobotPose();

        // collect the starting information
        IntoTheDeep.init_loop(telemetry, gamepad1);

        if (!wrist.isBusy()) {
            wrist.setPower(0.0);
        }

        // report the starting position
        telemetry.addData("pose", "%8.2f %8.2f %8.2f", Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);
    }

    @Override
    public void start() {
        // set the starting pose
        Motion.setPoseInches(-44, -60, 135.0);
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