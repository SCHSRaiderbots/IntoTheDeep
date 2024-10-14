@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
  private Gyroscope imu;
  private DcMotor motorTest;
  private DigitalChannel digitalTouch;
  private DistanceSensor sensorColorRange;
  private Servo servoTest;


  @Override
  public void runOpMode() {
    imu = hardwareMap.get(Gyroscope.class, "imu");
    motorTest = hardwareMap.get(DcMotor.class, "motorTest");
    digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
    sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
    servoTest = hardwareMap.get(Servo.class, "servoTest");
  
    leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
    leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
    rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

  
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      telemetry.addData("Status", "Running");
      telemetry.update();
      
      

      

    }
  }
}
