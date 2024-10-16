import android.util.Size;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import Config.ApriltagsFieldData;
import Config.DriveConstants;
import commands.Auto3PushCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;
import util.DashServer;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import util.DataListener;
import util.RobotDataServer;


@Autonomous
public class AutoOpmodeMecanumPathPlan extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;

    GyroEx gyro;

    private AprilTagProcessor webcamAprilTag;

    private Limelight3A limelightApriltag;

    private MecanumDriveSubsystem mecanumDriveSubsystem;
    
    private IntakeSubsystem intakeSubsystem;

    private static final boolean USE_DEBUG_FIELD_TAGS = true;

    double ACHIEVABLE_MAX_DISTANCE_PER_SECOND;

    private void initDriveWheels()
    {
        frontLeft = new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);//RPM_435
        frontRight = new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);//RPM_312
        backLeft = new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

//        frontLeft.setBuffer(1);
//        frontRight.setBuffer(1);
//        backLeft.setBuffer(1);
//        backRight.setBuffer(1);
        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.stopAndResetEncoder();
        frontRight.stopAndResetEncoder();
        backLeft.stopAndResetEncoder();
        backRight.stopAndResetEncoder();

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        backLeft.setRunMode(Motor.RunMode.VelocityControl);
        backRight.setRunMode(Motor.RunMode.VelocityControl);

        frontLeft.setVeloCoefficients(1.33, 0, 0.007);
        frontLeft.setFeedforwardCoefficients(0, 0.1);
        frontRight.setVeloCoefficients(1.31, 0, 0.007);
        frontRight.setFeedforwardCoefficients(0, 0.1);
        backLeft.setVeloCoefficients(1.31, 0, 0.007);
        backLeft.setFeedforwardCoefficients(0, 0.1);
        backRight.setVeloCoefficients(1.33, 0, 0.007);
        backRight.setFeedforwardCoefficients(0, 0.1);
		
	    frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        frontLeft.encoder.reset();
        frontRight.encoder.reset();
        backLeft.encoder.reset();
        backRight.encoder.reset();
        frontLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        frontRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);


        double ACHIEVABLE_MAX_TICKS_PER_SECOND = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        ACHIEVABLE_MAX_DISTANCE_PER_SECOND = ACHIEVABLE_MAX_TICKS_PER_SECOND * DriveConstants.DISTANCE_PER_PULSE;
        // about 1.567 m/s

        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initGyro()
    {
        gyro = new GyroEx() {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            @Override
            public void init() {
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetYaw();
            }

            @Override
            public double getHeading() {
                return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            @Override
            public double getAbsoluteHeading() {
                return 0;
            }

            @Override
            public double[] getAngles()
            {
                return new double[] {
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                        imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES),
                        imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES)
                };
            }

            @Override
            public com.arcrobotics.ftclib.geometry.Rotation2d getRotation2d() {
                return new com.arcrobotics.ftclib.geometry.Rotation2d(getHeading());
            }

            @Override
            public void reset() {
                imu.resetYaw();
            }

            @Override
            public void disable() {

            }

            @Override
            public String getDeviceType() {
                return "Internal IMU";
            }
        };

        gyro.init();
    }

    private void initWebCamAprilTag()
    {
        AprilTagLibrary apriltagLib;
        VisionPortal visionPortal;
        WebcamName apriltagCam;

        try{
            apriltagCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch(Exception e) {
            webcamAprilTag = null;
            telemetry.addLine(" Lost Webcam 1 /n");
            telemetry.update();
            return;
        }

        // Create the AprilTag processor.
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        if(USE_DEBUG_FIELD_TAGS)
        {
            AprilTagLibrary.Builder libBuilder = new AprilTagLibrary.Builder();
            libBuilder.addTag(ApriltagsFieldData.tag_2);
            libBuilder.addTag(ApriltagsFieldData.tag_42);
            myAprilTagProcessorBuilder.setTagLibrary(libBuilder.build());
        }
        else {
            myAprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());//libBuilder.build());////
        }

        // The following default settings are available to un-comment and edit as needed.
        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        
        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.
        myAprilTagProcessorBuilder.setDrawTagID(true);
        myAprilTagProcessorBuilder.setDrawTagOutline(true);
        myAprilTagProcessorBuilder.setDrawAxes(true);
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);
        myAprilTagProcessorBuilder.setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES);
        webcamAprilTag = myAprilTagProcessorBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(apriltagCam);
        builder.setCameraResolution(new Size(640,480)); //1280, 720));// fps 4
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);
        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);//YUY2);
        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);
        //builder.setLiveViewContainerId(0);

        // Set and enable the processor.
        builder.addProcessor(webcamAprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(webcamAprilTag, true);

    } 

    private void initLimelight()
    {
        try{
            limelightApriltag = hardwareMap.get(Limelight3A.class, "limelight");
        } catch(Exception e){
            limelightApriltag = null;
            telemetry.addLine(" Lost Limelight /n");
            telemetry.update();
            return;
        }

        limelightApriltag.pipelineSwitch(0);

        limelightApriltag.start();
    }

    private RobotDataServer dataServer;
    private void initDashServer()
    {
        telemetry.addData("Status", "Starting server...");
        telemetry.update();

        // Initialize the server and the listener to handle incoming data
        dataServer = new RobotDataServer(new DataListener() {
                @Override
                public void onDataReceived(String data) {
                    telemetry.addData("Data Received", data);
                    telemetry.update();
                }

                @Override
                public void onClientConnected() {
                    telemetry.addData("Status", "Client Connected.");
                    telemetry.update();
                }

                @Override
                public void onClientDisconnected() {
                    telemetry.addData("Status", "Client Disconnected.");
                    telemetry.update();
                }

                @Override
                public void onError(String errorMessage) {
                    telemetry.addData("Error", errorMessage);
                    telemetry.update();
                }
            });

        // Start the server on port 12345
        dataServer.startServer(12345);

        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    @Override
    public void initialize()
    {
        //telemetry.setMsTransmissionInterval(11);

        initDriveWheels();
        initGyro();
        initLimelight();
        initWebCamAprilTag();        

        intakeSubsystem = new IntakeSubsystem(telemetry);
        
        mecanumDriveSubsystem = new MecanumDriveSubsystem(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                gyro,
                webcamAprilTag,
                limelightApriltag,
                new edu.wpi.first.math.geometry.Pose2d(),
                telemetry, dataServer
        );

        mecanumDriveSubsystem.enableDrive();

        //wpiMecanumDriveSubsystem.setDefaultCommand(
        //ew ApriltagCommand(wpiMecanumDriveSubsystem, aprilTag, telemetry));

        // update telemetry every loop
        //schedule(new RunCommand(telemetry::update));

        schedule(new Auto3PushCommand(
                mecanumDriveSubsystem,
                intakeSubsystem,
                ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
                dataServer));
    }


    ////// DO NOT MODIFY THIS FUNCTION UNLESS YOU GET CONFIRMED!!!
    private int FrameCounter = 0;
    double startTime = 0;
    @Override
    public void runOpMode() {
        initDashServer();
        LynxModule controlHub  = hardwareMap.get(LynxModule.class, "Control Hub");
        // DashServer.Init(telemetry);
        // boolean connected = false;
        // do {
        //     connected = DashServer.Connect();
        //     connected |= DashServer.AddData("time", FrameCounter);
        //     sleep(1);
        // } while (!connected);
        initialize();
        waitForStart();
        startTime = (double) System.nanoTime() / 1E9;
        while (!isStopRequested() && opModeIsActive()) {
            double currentTime = (double) System.nanoTime() / 1E9 - startTime;
            // DashServer.AddData("OSTime", currentTime);
            dataServer.AddData("OSTime", currentTime);
            run();
            // DashServer.AddData("time", FrameCounter++);
            dataServer.AddData("time", FrameCounter++);
            // DashServer.AddData("busVoltage",
            //         controlHub .getInputVoltage(VoltageUnit.VOLTS));
            dataServer.AddData("bsVt",
                             controlHub .getInputVoltage(VoltageUnit.VOLTS));
            // DashServer.DashData();
            dataServer.DashData();

            sleep(5);
        }
        reset();
        if(limelightApriltag != null) limelightApriltag.stop();
        // DashServer.AddData("time", FrameCounter++);
        dataServer.AddData("time", FrameCounter++);
        // DashServer.AddData("OSTime", (double) System.nanoTime() / 1E9 - startTime);
        dataServer.AddData("OSTime", (double) System.nanoTime() / 1E9 - startTime);
        // DashServer.DashData();
        dataServer.DashData();
        // DashServer.Close();
        dataServer.stopServer();
    }
}
