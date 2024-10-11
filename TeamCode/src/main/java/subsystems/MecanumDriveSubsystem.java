package subsystems;

import android.annotation.SuppressLint;
import android.util.Size;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import Config.ApriltagsFieldData;
import Config.DriveConstants;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import util.DashServer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDriveSubsystem extends SubsystemBase
{
    private HardwareMap hardwareMap;
    private Motor frontLeft, backLeft, frontRight,  backRight;
    private GyroEx gyroEx;
    private AprilTagProcessor webcamApriltag;
    private Limelight3A limelightApriltag;
    private MecanumDriveKinematics mecanumDriveKinematics;

    private MecanumDriveWheelSpeeds currentWheelSpeeds = new MecanumDriveWheelSpeeds();
    private MecanumDriveWheelSpeeds targetWheelSpeeds = new MecanumDriveWheelSpeeds();
    private MecanumDriveWheelPositions mecanumDriveWheelPositions = new MecanumDriveWheelPositions();
    double ACHIEVABLE_MAX_DISTANCE_PER_SECOND;

    private MecanumDrivePoseEstimator mecanumPoseEstimator;
    private Pose2d currentEstimatedPose;

    boolean visionWebcamPoseEnable = false;
    Pose2d visionPose2dWebcam = new Pose2d();
    boolean visionLimelightPoseEnable = false;
    Pose2d visionPose2dLimelight = new Pose2d();
    private static final boolean USE_TESTING_TAGS = true;

    private Telemetry telemetry;
    private boolean telemetryEnable = false;

    public MecanumDriveSubsystem(
            HardwareMap hardwareMap,
            Pose2d initialPose,
            Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;

        initDriveWheels();
        initGyro();
        initLimelightAprilTag();
        initWebCamAprilTag();

        mecanumDriveKinematics = DriveConstants.kinematicsWPI;

        currentEstimatedPose = initialPose;

        //leftSpeed = gamepad::getLeftY;
        //rightSpeed = gamepad::getRightY;

        mecanumPoseEstimator = new MecanumDrivePoseEstimator(
                mecanumDriveKinematics,
                getGyroRotation2d(),
                getWheelDistance(),
                initialPose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        this.telemetry = telemetry;
    }

    private void initDriveWheels()
    {
        frontLeft = new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);//RPM_435
        frontRight = new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);
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

        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double ACHIEVABLE_MAX_TICKS_PER_SECOND = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        ACHIEVABLE_MAX_DISTANCE_PER_SECOND =
                ACHIEVABLE_MAX_TICKS_PER_SECOND * DriveConstants.DISTANCE_PER_PULSE;
        // about 1.567 m/s
    }

    private void initGyro()
    {
        gyroEx = new GyroEx() {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            @Override
            public void init() {
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                        RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  =
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
                RevHubOrientationOnRobot orientationOnRobot =
                        new RevHubOrientationOnRobot(logoDirection, usbDirection);

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

        gyroEx.init();
    }

    private void initWebCamAprilTag()
    {
        //https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_webcams/visionportal-webcams.html#arducam-global-shutter-120-fps
        //https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html#apriltag-programming

        AprilTagLibrary apriltagLib;
        VisionPortal visionPortal;
        WebcamName apriltagCam;

        try {
            apriltagCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e)
        {
            webcamApriltag= null;
            return;
        }

        // Create the AprilTag processor.
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        if(USE_TESTING_TAGS)
        {
            AprilTagLibrary.Builder libBuilder = new AprilTagLibrary.Builder();
            libBuilder.addTag(ApriltagsFieldData.tag_2);
            libBuilder.addTag(ApriltagsFieldData.tag_42);
            myAprilTagProcessorBuilder.setTagLibrary(libBuilder.build());
        }
        else {
            myAprilTagProcessorBuilder.setTagLibrary
                    (AprilTagGameDatabase.getCurrentGameTagLibrary());
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
        //myAprilTagProcessorBuilder.setLensIntrinsics(0,0,0,0);
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        webcamApriltag = myAprilTagProcessorBuilder.build();

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
        builder.addProcessor(webcamApriltag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(webcamApriltag, true);
    }

    private void initLimelightAprilTag()
    {
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FTC/pipelines
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/performing-charuco-camera-calibration
        
        try{
            limelightApriltag = hardwareMap.get(Limelight3A.class, "limelight");
        } catch(Exception e){limelightApriltag= null;}

        if(limelightApriltag != null){
            limelightApriltag.pipelineSwitch(0);

            limelightApriltag.start();
        }
    }

    @Override
    public void periodic()
    {
//        isSteopped = true;
        if(isSteopped)
        {
            telemetry.addData("STOPPED?? ", isSteopped);
        }
        else {
            // Get my wheel speeds; assume .getRate() has been
            // set up to return velocity of the encoder
            // in meters per second.
            currentWheelSpeeds.frontLeftMetersPerSecond = frontLeft.encoder.getRate();
            currentWheelSpeeds.frontRightMetersPerSecond = frontRight.encoder.getRate();
            currentWheelSpeeds.rearLeftMetersPerSecond = backLeft.encoder.getRate();
            currentWheelSpeeds.rearRightMetersPerSecond = backRight.encoder.getRate();


            mecanumDriveWheelPositions.frontLeftMeters = frontLeft.encoder.getDistance();
            mecanumDriveWheelPositions.frontRightMeters = frontRight.encoder.getDistance();
            mecanumDriveWheelPositions.rearLeftMeters = backLeft.encoder.getDistance();
            mecanumDriveWheelPositions.rearRightMeters = backRight.encoder.getDistance();

            // Update the pose
            currentEstimatedPose = mecanumPoseEstimator.update(
                    getGyroRotation2d(), mecanumDriveWheelPositions);

        }

        DashServer.AddData("curFrontLeftE", frontLeft.encoder.getRate());
        DashServer.AddData("curFrontRightE", frontRight.encoder.getRate());
        DashServer.AddData("curBackLeftE", backLeft.encoder.getRate());
        DashServer.AddData("curBackRightE", backRight.encoder.getRate());

        DashServer.AddData("curEstmPoseX", currentEstimatedPose.getX());
        DashServer.AddData("curEstmPoseY", currentEstimatedPose.getY());
        DashServer.AddData("curEstmPoseR", currentEstimatedPose.getRotation().getDegrees());


        //mecanumPoseEstimator.addVisionMeasurement();
        if (visionWebcamPoseEnable && (webcamApriltag != null)) {
            Pose2d botPose1 = visionWebcamUpdatePose();
            if (botPose1 != null) {
                visionPose2dWebcam = botPose1;
            }
        }
        if(visionLimelightPoseEnable && (limelightApriltag != null))
        {
            Pose2d botPose2 = visionLimelightUpdatePose();
            if(botPose2 != null) {
                visionPose2dLimelight = botPose2;
            }
        }

        if(telemetryEnable) {
            telemetry.addData("last Webcam Pose: ", visionPose2dWebcam);
            telemetry.addLine();
            telemetry.addData("last Limelight Pose: ", visionPose2dLimelight);
            telemetry.addLine();
            telemetry.addData("Robot Estimator Position: ", currentEstimatedPose);
            //mecanumPoseEstimator.getEstimatedPosition());
            telemetry.addLine();
            telemetry.addData("Asked Wheels Speed: ", targetWheelSpeeds);
            telemetry.addLine();
            telemetry.addData("Measured Wheels Speed: ", currentWheelSpeeds);
            //telemetry.addData("Robot Speed: ",
            // mecanumDriveKinematics.toChassisSpeeds(wheelSpeeds);
            //telemetry.addData("Robot Rotation: ", gyroEx.());
            //telemetryAprilTag();
            telemetry.update();
        }
    }

    public void driveBySpeedEvent(MecanumDriveWheelSpeeds speedsCommand)
    {
        targetWheelSpeeds.rearRightMetersPerSecond = speedsCommand.frontRightMetersPerSecond;
        targetWheelSpeeds.rearLeftMetersPerSecond = speedsCommand.frontLeftMetersPerSecond;
        targetWheelSpeeds.frontLeftMetersPerSecond = speedsCommand.frontLeftMetersPerSecond;
        targetWheelSpeeds.frontRightMetersPerSecond = speedsCommand.frontRightMetersPerSecond;
        targetWheelSpeeds.desaturate(DriveConstants.MAX_VELOCITY);

        /// DriveConstants.MAX_VELOCITY);
        frontLeft.set( targetWheelSpeeds.frontLeftMetersPerSecond);
        frontRight.set( targetWheelSpeeds.frontRightMetersPerSecond);
        backLeft.set( targetWheelSpeeds.rearLeftMetersPerSecond);
        backRight.set( targetWheelSpeeds.rearRightMetersPerSecond);

        DashServer.AddData("askFrontLeft", targetWheelSpeeds.frontLeftMetersPerSecond);
        DashServer.AddData("askFrontRight", targetWheelSpeeds.frontRightMetersPerSecond);
        DashServer.AddData("askBackLeft", targetWheelSpeeds.rearLeftMetersPerSecond);
        DashServer.AddData("askBackRight", targetWheelSpeeds.rearRightMetersPerSecond);
    }

    public void enableDrive()
    {
        isSteopped = false;
    }
	static boolean isSteopped = false;
    public void stopDrive()
    {
        telemetry.addLine(" Request to STOP!! /n");
        telemetry.update();
        frontLeft.set( 0);
        frontRight.set( 0);
        backLeft.set( 0);
        backRight.set( 0);
        frontLeft.motor.setPower(0);
        frontRight.motor.setPower(0);
        backLeft.motor.setPower(0);
        backRight.motor.setPower(0);

        isSteopped = true;
    }
    public void subsystemExit()
    {
        if(limelightApriltag != null) limelightApriltag.stop();
    }

    private Pose2d visionWebcamUpdatePose()
    {
        Pose2d ret = null;

        List<AprilTagDetection> currentDetections = webcamApriltag.getDetections();
        //getFreshDetections();//.getDetections();

        if(telemetryEnable) {
            telemetry.addData("# AprilTags Detected: ", currentDetections.size());
        }

        if(currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == ApriltagsFieldData.tag_2.id)
                    {
                        Pose3d tagFieldPose = new Pose3d(new Translation3d(0, 0, 0.07),
                                new Rotation3d(0, 0, 0));
                        Transform3d tagToCamera = new Transform3d(
                                new Translation3d(detection.ftcPose.y,
                                        -1 * detection.ftcPose.x,
                                        detection.ftcPose.z),
                                new Rotation3d(Units.degreesToRadians(detection.ftcPose.roll),
                                        Units.degreesToRadians(detection.ftcPose.pitch),
                                        Units.degreesToRadians(detection.ftcPose.yaw)));

                        Pose3d cameraFieldPose = ComputerVisionUtil.objectToRobotPose(
                                tagFieldPose, tagToCamera, DriveConstants.CamToRobot);
                        //tagFieldPose.transformBy(tagToCamera);

                        ret = new Pose2d(-1 * cameraFieldPose.getX(),
                                -1 * cameraFieldPose.getY(),
                                new Rotation2d(Units.degreesToRadians(-1 * detection.ftcPose.yaw)));

                        double timeAcquisition = detection.frameAcquisitionNanoTime / 1e-9;

                        setStdDevsWebcamPose(ret);
                        mecanumPoseEstimator.addVisionMeasurement( ret, timeAcquisition);

                        DashServer.AddData("WebcamX", ret.getX());
                        DashServer.AddData("WebcamY", ret.getY());
                        DashServer.AddData("WebcamR", ret.getRotation().getDegrees());
                        DashServer.AddData("WCtime", timeAcquisition);

                        if(telemetryEnable) {
                            //                            telemetry.addLine(String.format("ComputerVisionUtil %6.3f %6.3f %6.1f  (xyr)", -1*cameraFieldPose.getX(), -1*cameraFieldPose.getY(),
//                                    -1*detection.ftcPose.yaw));
//
//                            telemetry.addLine(String.format("==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.1f  (meter)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.2f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//
//                telemetry.addLine(String.format("BOT_Bear %6.3f %6.3f %6.1f  (xyr)",
//                        Math.cos( detection.ftcPose.bearing+detection.ftcPose.yaw)*detection.ftcPose.range,
//                        Math.sin( detection.ftcPose.bearing+detection.ftcPose.yaw)*detection.ftcPose.range,
//                        -1*detection.ftcPose.yaw));
//
//
//                // x and r ok!! need to calculate "y" by detection.ftcPose.bearing's +/-!! TODO jd
//                telemetry.addLine(String.format("BOT %6.3f %6.3f %6.1f  (xyr)", detection.ftcPose.y, detection.ftcPose.x,
//                        -1*detection.ftcPose.yaw)); // need to transform to field angle? JD
//
//
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                            //telemetry.addLine("Camera FPS: " + df.format(camera.getFps()));
                            //telemetry.addLine("Max theoretical FPS: " + df.format(camera.getCurrentPipelineMaxFps()));
                        }

                    }
                }
            }
        }

        return ret;
    }

    private Pose2d visionLimelightUpdatePose()
    {
        Pose2d ret = null;

        LLStatus status = limelightApriltag.getStatus();
        if(telemetryEnable) {
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
        }

        LLResult result = limelightApriltag.getLatestResult();
        if (result != null) {
            // Access general information
            org.firstinspires.ftc.robotcore.external.navigation.Pose3D botpose = 
                result.getBotpose();
            if(telemetryEnable) {
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            }
            
            if (result.isValid()) {
                if(telemetryEnable) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }

                ret = new Pose2d(botpose.getPosition().x, botpose.getPosition().y,
                        new Rotation2d(botpose.getOrientation().getYaw()));

                setStdDevsLimelightPose(ret);

                mecanumPoseEstimator.addVisionMeasurement(ret,result.getControlHubTimeStamp()/1000.0);

                DashServer.AddData("LimelightX", botpose.getPosition().x);
                DashServer.AddData("LimelightY", botpose.getPosition().y);
                DashServer.AddData("LimelightR", botpose.getOrientation().getYaw());
                DashServer.AddData("LLtime", result.getControlHubTimeStamp()/1000.0);
            }
        } else {
            if(telemetryEnable) {
                telemetry.addData("Limelight", "No data available");
            }
        }
        return ret;
    }

    private void setStdDevsWebcamPose(Pose2d visionPose)
    {
        //frc example poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        double xyStds = 0.5;
        double degStds = 999999;

        if(visionPose != null)
        {
            // trust it more <-> less
            xyStds = 0.5;//1.0;
            degStds = 6;//12;
        }
        mecanumPoseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

//                            && (detection.ftcPose.z > -0.2)
//                            && (detection.ftcPose.z < 0.2)
//                            && (detection.ftcPose.range < 2.0)


//        if(!receivedValidData)
//            doRejectUpdate = true;
//        else if(botPose1.getZ() > 0.3 || botPose1.getZ() < -0.3)
//            doRejectUpdate = true;

        //JD TODO
//        else if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
//        {
//            if(mt1.rawFiducials[0].ambiguity > .7)
//            {
//                doRejectUpdate = true;
//            }
//            if(mt1.rawFiducials[0].distToCamera > 3)
//            {
//                doRejectUpdate = true;
//            }
//
//
//            // 1 target with large area and close to estimated pose
//            if (mt1.avgTagArea > 0.8 && mt1.rawFiducials[0].distToCamera < 0.5) {
//                xyStds = 1.0;
//                degStds = 12;
//            }
//            // 1 target farther away and estimated pose is close
//            else if (mt1.avgTagArea > 0.1 && mt1.rawFiducials[0].distToCamera < 0.3) {
//                xyStds = 2.0;
//                degStds = 30;
//            }
//        }
//        else if (mt1.tagCount >= 2) {
//            xyStds = 0.5;
//            degStds = 6;
//        }
        //mecanumPoseEstimator.setVisionMeasurementStdDevs();
    }

    private void setStdDevsLimelightPose(Pose2d visionPose)
    {
        //mecanumPoseEstimator.setVisionMeasurementStdDevs();
    }

    public Pose2d getCurrentEstimatedPose()
    {
        return mecanumPoseEstimator.getEstimatedPosition();
    }

    public MecanumDriveKinematics getKinematics()
    {
        return mecanumDriveKinematics;
    }

    private MecanumDriveWheelPositions getWheelDistance() {
        return mecanumDriveWheelPositions;
    }

    public Rotation2d getGyroRotation2d() {
        return new Rotation2d(gyroEx.getHeading());
    }

    public double getAchievableMaxSpeed()
    {
        return ACHIEVABLE_MAX_DISTANCE_PER_SECOND;
    }

//    public void driveByVoltage(MecanumDriveMotorVoltages mecanumDriveMotorVoltages)
//    {
//        frontLeft. ctrl power
//        frontLeft.set( mecanumDriveMotorVoltages.frontLeftVoltage);
//        frontRight.set( mecanumDriveMotorVoltages.frontRightVoltage);
//        backLeft.set( mecanumDriveMotorVoltages.rearLeftVoltage);
//        backRight.set( mecanumDriveMotorVoltages.rearRightVoltage);
//    }
}
