package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.tileX;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.SequentialCommandGroup;

@Autonomous(name="Auto Wave", group="testing")
@SuppressWarnings("unused")
public class AutoWave extends OpMode {
    Command command;
    Vision vision;
    // Arm arm;
    // Wrist wrist;
    // Gripper gripper;

    @Override
    public void init() {
        // identify the robot
        Motion.robot = RobotId.identifyRobot(hardwareMap);
        Motion.init(hardwareMap);

        vision = new Vision(hardwareMap);

        // arm = new Arm(hardwareMap);
        // wrist = new Wrist(hardwareMap);
        // gripper = new Gripper(hardwareMap);

        IntoTheDeep.init();
    }

    @Override
    public void init_loop() {
        // figure our position
        Motion.updateRobotPose();

        // collect the starting information
        IntoTheDeep.init_loop(telemetry, gamepad1);

        // report the starting position
        telemetry.addData("pose", "%8.2f %8.2f %8.2f", Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);
    }

    @Override
    public void start() {
        // make the command
        command = new SequentialCommandGroup(
                // new Delay(0.5),
                // drive forward
                new DriveForward( tileX(2, 6.0))
                // new DriveTurnToward(tileXR(2.5), tileYR(-0.5)),
                // new DriveTo(tileX(2.5), tileYR(-0.5)),
                // new MoveArm(0, arm)
        );

        command.initialize();
        command.execute();
    }

    @Override
    public void loop() {
        // figure our position
        Motion.updateRobotPose();

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