
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

/**
 * Class that keeps the vision code in one place.
 * The OpModes that want to use vision should call the methods in this class.
 * Steps are
 *   vision = new Vision(HardwareMap);
 */
public class Vision {
    /** A locale used to format strings */
    final Locale locale = new Locale("en-US");

    /** the webcam */
    WebcamName webcamName;

    private static final boolean USE_WEBCAM = true;

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     * <p>
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     * <p>
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     * <p>
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     * <p>
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            -3.875, 7.0, 5.0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * the AprilTag processor
     */
    private AprilTagProcessor aprilTag;

    /**
     * The Vision Portal
     *   visionPortal.stopStreaming()
     *   visionPortal.resumeStreaming()
     *   visionPortal.setProcessorEnabled(tfod, true)
     *   visionPortal.setProcessorEnabled(aprilTag, true)
     *   visionPortal.close() when done
     */
    private VisionPortal visionPortal;

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // define some constants and conversions here
    static final float mmPerInch        = 25.4f;
    // the height of the center of the target image above the floor
    // TODO: check the dimensions
    // https://firstinspiresst01.blob.core.windows.net/first-energize-ftc/field-setup-and-assembly-guide.pdf
    // page 22 says the horizontal center line is 6.375 from the floor or 5.75 inches from top of the tile
    private static final float mmTargetHeight   = 6 * mmPerInch;
    // TODO: these values are slightly off
    private static final float halfField        = 72 * mmPerInch;
    // the pitch of the tiles is 23 5/8
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // these are hack values for updating the pose
    static double inchX = 0;
    static double inchY = 0;
    static double degTheta = 0;

    /*
     * CenterStage model assets
     *   0: pixel
     *
     * PowerPlay model assets
     *  PowerPlay.tflite 0: Bolt, 1: Bulb, 2: Panel,
     *
     * FreightFrenzy model assets:
     *  FreightFrenzy_BCDM.tflite 0: Ball, 1: Cube, 2: Duck, 3: Marker (duck location marker).
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    // private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    public Vision (HardwareMap hardwareMap) {
        // build aprilTag
        initAprilTag();

        // build VisionProcessor
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        // Set confidence threshold for TFOD recognitions, at any time.
        // tfod.setMinResultConfidence(0.2f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);
    }

    /**
     * Method to enable or disable vision processing.
     * @param enabled
     */
    void enableAprilTags(boolean enabled) {
        // enable or disable vision
        visionPortal.setProcessorEnabled(aprilTag, enabled);
    }

    /**
     * Initialize the AprilTag processor.
     * side-effects aprilTag
     */
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // I do not see a way of getting a list of possible tags....
        // aprilTag.setPoseSolver();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Report the AprilTag id and its name
                telemetry.addLine(String.format(locale, "\n==== (ID %d) %s", detection.id, detection.metadata.name));

                // Report the calculated robotPose
                // the Pose3D robotPose is extracted for us.
                // The pose information is in inches!
                Pose3D robotPose = detection.robotPose;
                Position robotPosition = robotPose.getPosition();
                YawPitchRollAngles robotOrientation = robotPose.getOrientation();
                telemetry.addLine(String.format(locale, "robot pose %6.1f %6.1f %6.1f. Yaw %6.1f (deg)",
                        robotPosition.x, robotPosition.y, robotPosition.z, robotOrientation.getYaw()));
                // camera position is messed up, so report all the angles
                telemetry.addLine(String.format(locale, "robot YPR %6.1f %6.1f %6.1f (deg)",
                        robotOrientation.getYaw(), robotOrientation.getPitch(), robotOrientation.getRoll()));

                // Report the AprilTagPoseFtc ftcPose
                AprilTagPoseFtc ftcPose = detection.ftcPose;
                telemetry.addLine(String.format(locale, "XYZ %6.1f %6.1f %6.1f  (inch)", ftcPose.x, ftcPose.y, ftcPose.z));
                telemetry.addLine(String.format(locale, "PRY %6.1f %6.1f %6.1f  (deg)", ftcPose.pitch, ftcPose.roll, ftcPose.yaw));
                telemetry.addLine(String.format(locale, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", ftcPose.range, ftcPose.bearing, ftcPose.elevation));
            } else {
                // No metadata. Report the AprilTag id and its location
                telemetry.addLine(String.format(locale, "\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(locale, "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
