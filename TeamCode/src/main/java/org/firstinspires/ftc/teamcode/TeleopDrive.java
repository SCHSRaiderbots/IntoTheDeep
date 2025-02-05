package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// import java.util.Locale;

/**
 * TeleOp mode for competition.
 */
@TeleOp(name="Teleop Drive", group ="Competition")
@SuppressWarnings("unused")
public class TeleopDrive extends OpMode {

    // the Vision object
    Vision vision = null;

    Elevator elevator = null;

    Wrist wrist = null;

    Gripper gripper;

    Tray tray;

    SampleDetector sampler;

    // Whether or not to use the IMU
    boolean bIMU = false;

    // The IMU sensor object
    // BNO055IMU imu = null;
    // there is a new IMU object...
    IMU imu = null;

    /** the LynxModule serial number */
    String strSerialNumber;

    @Override
    public void init() {
        // get the serial number
        // TODO: use serial number to identify robot?
        strSerialNumber = LogDevice.getSerialNumber(hardwareMap);

        // report the LynxModules
        LogDevice.dumpFirmware(hardwareMap);

        // try to identify the robot
        robot = RobotId.identifyRobot(hardwareMap, RobotId.ROBOT_2023);

        // identify the robot -- we are expecting 2022 for this test
        Log.d("Identify", robot.toString());

        // initialize motion
        Motion.init(hardwareMap);

        // create the vision object
        if (robot == RobotId.ROBOT_2022 || robot == RobotId.ROBOT_2023) {
            // TODO: currently uses ROBOT_2022 camera offset!
            vision = new Vision(hardwareMap);

            // get the Sample detector
            sampler = new SampleDetector(hardwareMap);

            // get the gripper
            gripper = new Gripper(hardwareMap);

            // get thw tray
            tray = new Tray(hardwareMap);
        }

        // create the subsystems
        if (robot == RobotId.ROBOT_2023) {
            elevator = new Elevator(hardwareMap);

            wrist = new Wrist(hardwareMap);
        }

        if (bIMU) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.

            /* The next two lines define Hub orientation.
             * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
             *
             * To Do:  EDIT these two lines to match YOUR mounting configuration.
             */
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(parameters);
        }
    }

    @Override
    public void init_loop() {
        // report the serial number during init
        // this causes an update, so it will flash the display
        // telemetry.addData("Serial Number", strSerialNumber);

        // update the robot pose
        Motion.updateRobotPose();
    }

    @Override
    public void start() {
        // report current status

        // Motion.setPoseInches(0,0,0);

        // run using encoder
        Motion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (vision != null) {
            vision.enableAprilTags(true);
        }
    }

    @Override
    public void loop() {
        // update the robot pose
        Motion.updateRobotPose();
        Motion.reportPosition(telemetry);

        // TODO: 2022 robot reports as ROBOT_2023!
        telemetry.addData("Robot", robot);

        if (vision != null) {
            vision.telemetryAprilTag(telemetry);
        }

        if (bIMU) {
            reportIMU();
        }

        // now process the controls...

        // do some driving was -0.7
        // The minus sign is because stick pushed forward is negative.
        double forward = -1.0 * boost(gamepad1.left_stick_y);
        double turn = 0.4 * (gamepad1.right_stick_x);

        // max tick velocity should 6000 RPM * 28 ticks per rev = 2800
        double rpm = 6000.0;
        double v = (rpm / 60) * Motion.HD_HEX_TICKS_PER_REV;
        Motion.setVelocity(v * (forward+turn), v * (forward-turn));

        if (gripper != null) {
            gripper.grip(gamepad2.left_stick_y);
        }

        if (tray != null) {
            tray.setPosition(0.5 + 0.5 * gamepad2.right_stick_y);
        }

        // if we have a wrist...
        if (wrist != null) {
            if (gamepad2.dpad_down) {
                wrist.setPosition(Wrist.WristPosition.GROUND);
            }
            if (gamepad2.dpad_left) {
                wrist.setPosition(Wrist.WristPosition.HORIZ);
            }
            if (gamepad2.dpad_right) {
                wrist.setPosition(Wrist.WristPosition.SKIM);
            }
            if (gamepad2.dpad_up) {
                wrist.setPosition(Wrist.WristPosition.BASKET);
            }

        }

        if (gamepad1.y) {
            // set the pose
            Motion.setPoseInches(Vision.inchX, Vision.inchY, Vision.degTheta);
        }

        // if we have an elevator
        if (elevator != null && wrist != null) {
            if (gamepad2.a) {
                elevator.setTargetPosition(Elevator.ElevatorPosition.BOTTOM);    // was 800
                wrist.setPosition(Wrist.WristPosition.VERTICAL);
            }
            if (gamepad2.b) {
                elevator.setTargetPosition(Elevator.ElevatorPosition.LOW_NET);   // was 1900
                wrist.setPosition(Wrist.WristPosition.VERTICAL);
            }
            if (gamepad2.x) {
                elevator.setTargetPosition(Elevator.ElevatorPosition.TRANSFER);
                wrist.setPosition(Wrist.WristPosition.BASKET);
            }
            if (gamepad2.y) {
                elevator.setTargetPosition(Elevator.ElevatorPosition.HIGH_NET);
                wrist.setPosition(Wrist.WristPosition.VERTICAL);
            }
            telemetry.addData("Elevator Position", "%d units", elevator.getPosition());
        }

        // if we have a SampleDetector
        if (sampler != null) {
            telemetry.addData("Sample Color", "%s", sampler.getColor());
        }
    }

    /**
     * Square a value retaining the sign
     * @param x value from -1 to 1
     * @return x * abs(x)
     */
    private double boost(double x) {
        return x * Math.abs(x);
    }

    @Override
    public void stop() {
        // turn off tracking
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void reportIMU() {
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    /*
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
     */
}