package subsystems;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import Config.ApriltagsFieldData;
import Config.DriveConstants;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import util.DashServer;
import util.OldDriverFilter2;
import util.RobotDataServer;
import util.filters.DeadbandFilter;
import util.filters.FilterSeries;
import util.filters.ScaleFilter;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


public class MecanumDriveSubsystem extends SubsystemBase
{
    Motor frontLeft, backLeft, frontRight,  backRight;
    Motor.Encoder frontLeft_encoder, backLeft_encoder, frontRight_encoder, backRight_encoder;
    GyroEx gyroEx;
    AprilTagProcessor webcamApriltag;
    Limelight3A limelightApriltag;
    MecanumDriveKinematics mecanumDriveKinematics;

    MecanumDriveWheelSpeeds currentWheelSpeeds;

    private MecanumDrivePoseEstimator mecanumPoseEstimator;
    private Pose2d currentEstimatedPose;


    Pose2d visionPose2dWebcam = new Pose2d();
    Pose2d visionPose2dLimelight = new Pose2d();
    private static final boolean USE_TESTING_TAGS = true;

    private Telemetry telemetry;

    private boolean telemetryEnable = true;

    boolean visionLimelightPoseEnable = false;
    boolean visionWebcamPoseEnable = false;
    private PIDController headingTurnPID = new PIDController(0.01, 0.0, 0.000001);

    public MecanumDriveSubsystem(
            Motor frontLeft,
            Motor frontRight,
            Motor backLeft,
            Motor backRight,
            GyroEx gyro,
            AprilTagProcessor webcamApriltag,
            Limelight3A limelightApriltag,
            Pose2d initialPose,
            Telemetry telemetry//,
            //RobotDataServer dataServer
    )
    {
        m_frontLeftMotor = frontLeft::set;
        m_rearLeftMotor = backLeft::set;
        m_frontRightMotor = frontRight::set;
        m_rearRightMotor = backRight::set;

        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;

        this.frontLeft_encoder = frontLeft.encoder;
        this.backLeft_encoder = backLeft.encoder;
        this.frontRight_encoder = frontRight.encoder;
        this.backRight_encoder = backRight.encoder;

        gyroEx = gyro;

        mecanumDriveKinematics = DriveConstants.kinematicsWPI;

        this.webcamApriltag = webcamApriltag;
        this.limelightApriltag = limelightApriltag;

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
        //this.dataServer = dataServer;


        headingTurnPID.enableContinuousInput(-180, 180);
        headingTurnPID.setTolerance(0.5);
    }

//    com.arcrobotics.ftclib.geometry.Rotation2d previousHeading = new com.arcrobotics.ftclib.geometry.Rotation2d();
//    com.arcrobotics.ftclib.geometry.Rotation2d currentHeading = new com.arcrobotics.ftclib.geometry.Rotation2d();

    double currentHeadingPi2NPi = 0;

    @Override
    public void periodic()
    {
//        previousHeading = currentHeading;
//        currentHeading = gyroEx.getRotation2d();
//
//        double cos_angle = previousHeading.getCos();
//                double sin_angle = previousHeading.getSin();
//        com.arcrobotics.ftclib.geometry.Rotation2d a =
//                new com.arcrobotics.ftclib.geometry.Rotation2d(cos_angle, -sin_angle);
//        currentHeadingPi2NPi = a.rotateBy(currentHeading).getDegrees();

//        isSteopped = true;
        if(isSteopped)
        {
            if(telemetryEnable) {
                telemetry.addData("STOPPED?? ", isSteopped);
            }
        }
        else {
            // Get my wheel speeds; assume .getRate() has been
            // set up to return velocity of the encoder
            // in meters per second.
            currentWheelSpeeds = new MecanumDriveWheelSpeeds
            (
                    frontLeft_encoder.getRate(), frontRight_encoder.getRate(),
                    backLeft_encoder.getRate(), backRight_encoder.getRate()
            );

            MecanumDriveWheelPositions mecanumDriveWheelPositions = new MecanumDriveWheelPositions(
                    frontLeft_encoder.getDistance(), frontRight_encoder.getDistance(),
                    backLeft_encoder.getDistance(), backRight_encoder.getDistance());

            // Update the pose
            currentEstimatedPose = mecanumPoseEstimator.updateWithTime(
                    (double) System.nanoTime() / 1E9,
                    getGyroRotation2d(), mecanumDriveWheelPositions);

            currentHeadingPi2NPi = currentEstimatedPose.getRotation().getDegrees();

        }
        DashServer.AddData("dsEtmTime", (double) System.nanoTime() / 1E9);//(double)System.currentTimeMillis()/1000.0);//

        DashServer.AddData("dsFLspd", frontLeft_encoder.getRate());
        DashServer.AddData("dsFRspd", frontRight_encoder.getRate());
        DashServer.AddData("dsBLspd", backLeft_encoder.getRate());
        DashServer.AddData("dsBRspd", backRight_encoder.getRate());

//        DashServer.AddData("dsPoseX", currentEstimatedPose.getX());
//        DashServer.AddData("dsPoseY", currentEstimatedPose.getY());
//        DashServer.AddData("dsPoseR", currentEstimatedPose.getRotation().getDegrees());

        //mecanumPoseEstimator.addVisionMeasurement();
        if (visionWebcamPoseEnable && (webcamApriltag != null))
        {
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
            telemetry.addLine();
            telemetry.addData("Asked Wheels Speed: ", askedWheelSpeeds);
            telemetry.addLine();
            telemetry.addData("Measured Wheels Speed: ", currentWheelSpeeds);
            telemetry.addLine();
            telemetry.addData("Limited Heading: ", currentHeadingPi2NPi);
            telemetry.addData("FieldOriented: ", fieldRelative);
            telemetry.update();
        }
    }

    MecanumDriveWheelSpeeds askedWheelSpeeds = new MecanumDriveWheelSpeeds();
    public void driveBySpeedEvent(MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds)
    {
        mecanumDriveWheelSpeeds.desaturate(DriveConstants.MAX_VELOCITY);
        frontLeft.set( mecanumDriveWheelSpeeds.frontLeftMetersPerSecond);/// DriveConstants.MAX_VELOCITY);
        frontRight.set( mecanumDriveWheelSpeeds.frontRightMetersPerSecond);/// DriveConstants.MAX_VELOCITY);
        backLeft.set( mecanumDriveWheelSpeeds.rearLeftMetersPerSecond);/// DriveConstants.MAX_VELOCITY);
        backRight.set( mecanumDriveWheelSpeeds.rearRightMetersPerSecond);/// DriveConstants.MAX_VELOCITY);

        askedWheelSpeeds.frontLeftMetersPerSecond = mecanumDriveWheelSpeeds.frontLeftMetersPerSecond;
        askedWheelSpeeds.frontRightMetersPerSecond = mecanumDriveWheelSpeeds.frontRightMetersPerSecond;
        askedWheelSpeeds.rearLeftMetersPerSecond = mecanumDriveWheelSpeeds.rearLeftMetersPerSecond;
        askedWheelSpeeds.rearRightMetersPerSecond = mecanumDriveWheelSpeeds.rearRightMetersPerSecond;

        DashServer.AddData("dsTgtFLspd", askedWheelSpeeds.frontLeftMetersPerSecond);
        DashServer.AddData("dsTgtFRspd", askedWheelSpeeds.frontRightMetersPerSecond);
        DashServer.AddData("dsTgtBLspd", askedWheelSpeeds.rearLeftMetersPerSecond);
        DashServer.AddData("dsTgtBRspd", askedWheelSpeeds.rearRightMetersPerSecond);
    }

	public void enableDrive()
    {
        isSteopped = false;
    }
	static boolean isSteopped = false;
    public void stopDrive()
    {
        if(telemetryEnable) {
            telemetry.addLine(" Request to STOP!! /n");
        }
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
        //powered USB hub is neeeded
        //https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_uvc_camera/configuring-uvc-camera.html
        Pose2d ret = null;

        List<AprilTagDetection> currentDetections = webcamApriltag.getFreshDetections();
        //getFreshDetections();//.getDetections();

        if(telemetryEnable) {
            telemetry.addData("# AprilTags Detected: ", currentDetections.size());
        }

        if(currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == ApriltagsFieldData.tag_2.id)
                    {
                        Pose3d tagFieldPose = new Pose3d(new Translation3d(-0.61, 0, 0.07),
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

                        double timeAcquisition = (double)detection.frameAcquisitionNanoTime / 1E9;

                        //setStdDevsWebcamPose(ret);
                        mecanumPoseEstimator.addVisionMeasurement( ret, timeAcquisition);

                        DashServer.AddData("WebcamX", ret.getX());
                        DashServer.AddData("WebcamY", ret.getY());
                        DashServer.AddData("WebcamR", ret.getRotation().getDegrees());
                        DashServer.AddData("WebcamT", timeAcquisition);

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

    private static final boolean USE_3D_MAGTAG2 = false;
    private Pose2d visionLimelightUpdatePose()
    {
        double radians = 0;
        if(USE_3D_MAGTAG2) {
            radians = gyroEx.getHeading();
            limelightApriltag.updateRobotOrientation(Math.toDegrees(radians));
        }

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

            if(telemetryEnable) {
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            }

            if (result.isValid()) {
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D botpose;
                if(USE_3D_MAGTAG2)
                    botpose = result.getBotpose_MT2();
                else
                    botpose = result.getBotpose();

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
                        new Rotation2d(Math.toRadians(botpose.getOrientation().getYaw())));
                //double timeAcquisition = (double) result.getControlHubTimeStamp()/1000.0;
                double timeAcquisition = (double)(System.nanoTime() - (result.getStaleness()*1000000)) / 1E9;
//                setStdDevsLimelightPose(ret);
//                mecanumPoseEstimator.addVisionMeasurement(ret,timeAcquisition);

                DashServer.AddData("LlX", ret.getX());
                DashServer.AddData("LlY", ret.getY());
                DashServer.AddData("LlR", ret.getRotation().getDegrees());
                DashServer.AddData("LlT", timeAcquisition);
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
        return new MecanumDriveWheelPositions(
                frontLeft_encoder.getDistance(),
                backLeft_encoder.getDistance(),
                frontRight_encoder.getDistance(),
                backRight_encoder.getDistance());
    }

    public Rotation2d getGyroRotation2d() {
        return new Rotation2d(gyroEx.getRotation2d().getRadians());
    }

    public double getAchievableMaxSpeed()
    {
        double ACHIEVABLE_MAX_TICKS_PER_SECOND = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        double ACHIEVABLE_MAX_DISTANCE_PER_SECOND =
                ACHIEVABLE_MAX_TICKS_PER_SECOND * DriveConstants.DISTANCE_PER_PULSE;
        // about 1.567 m/s
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

    public void setVisionLimelightPoseEnable(boolean en)
    {
        visionLimelightPoseEnable = en;
    }


    /**********************************************************************
     * The code below are used for TeleOp Mode
     ********************************************************************+*/
    /** Default input deadband. */
    public static final double kDefaultDeadband = 0.02;

    /** Default maximum output. */
    public static final double kDefaultMaxOutput = 1.0;

    /** Input deadband. */
    protected double m_deadband = kDefaultDeadband;

    /** Maximum output. */
    protected double m_maxOutput = kDefaultMaxOutput;

    /** The location of a motor on the robot for the purpose of driving. */
    public enum MotorType {
        /** Front left motor. */
        kFrontLeft(0),
        /** Front right motor. */
        kFrontRight(1),
        /** Rear left motor. */
        kRearLeft(2),
        /** Reat right motor. */
        kRearRight(3),
        /** Left motor. */
        kLeft(0),
        /** Right motor. */
        kRight(1),
        /** Back motor. */
        kBack(2);

        /** MotorType value. */
        public final int value;

        MotorType(int value) {
            this.value = value;
        }
    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured counterclockwise from the positive X axis. The robot's speed is
     * independent of its angle or rotation rate.
     *
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
     *     positive.
     */
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        driveCartesian(xSpeed, ySpeed, zRotation, 0);//new Rotation2d());
    }

    private final DoubleConsumer m_frontLeftMotor;
    private final DoubleConsumer m_rearLeftMotor;
    private final DoubleConsumer m_frontRightMotor;
    private final DoubleConsumer m_rearRightMotor;

    // Used for Sendable property getters
    private double m_frontLeftOutput;
    private double m_rearLeftOutput;
    private double m_frontRightOutput;
    private double m_rearRightOutput;

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured counterclockwise from the positive X axis. The robot's speed is
     * independent of its angle or rotation rate.
     *
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
     *     positive.
     * @param gyroAngle The gyro heading around the Z axis. Use this to implement field-oriented
     *     controls.
     */
    public void driveCartesian(double xSpeed, double ySpeed,
                               double zRotation, double gyroAngle) {

        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);

        MecanumDriveWheelSpeeds speeds = driveCartesianIK(xSpeed, ySpeed, zRotation, gyroAngle);

        m_frontLeftOutput = speeds.frontLeftMetersPerSecond * m_maxOutput;
        m_rearLeftOutput = speeds.rearLeftMetersPerSecond * m_maxOutput;
        m_frontRightOutput = speeds.frontRightMetersPerSecond * m_maxOutput;
        m_rearRightOutput = speeds.rearRightMetersPerSecond * m_maxOutput;

        m_frontLeftMotor.accept(m_frontLeftOutput);
        m_frontRightMotor.accept(m_frontRightOutput);
        m_rearLeftMotor.accept(m_rearLeftOutput);
        m_rearRightMotor.accept(m_rearRightOutput);

        DashServer.AddData("tlpFlPower", m_frontLeftOutput);
        DashServer.AddData("tlpRlPower", m_rearLeftOutput);
        DashServer.AddData("tlpFrPower", m_frontRightOutput);
        DashServer.AddData("tlpRrPower", m_rearRightOutput);
    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured counterclockwise from straight ahead. The speed at which the robot
     * drives (translation) is independent of its angle or rotation rate.
     *
     * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
     * @param angle The gyro heading around the Z axis at which the robot drives.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
     *     positive.
     */
    public void drivePolar(double magnitude, Rotation2d angle, double zRotation) {
        driveCartesian(
                magnitude * angle.getCos(), magnitude * angle.getSin(),
                zRotation, 0);//new Rotation2d());
    }

    /**
     * Cartesian inverse kinematics for Mecanum platform.
     *
     * <p>Angles are measured counterclockwise from the positive X axis. The robot's speed is
     * independent of its angle or rotation rate.
     *
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
     *     positive.
     * @return Wheel speeds [-1.0..1.0].
     */
    public MecanumDriveWheelSpeeds driveCartesianIK(double xSpeed, double ySpeed, double zRotation) {
        return driveCartesianIK(xSpeed, ySpeed, zRotation, 0);//new Rotation2d());
    }

    /**
     * Cartesian inverse kinematics for Mecanum platform.
     *
     * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent of
     * its angle or rotation rate.
     *
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
     *     positive.
     * @param gyroAngle The gyro heading around the Z axis. Use this to implement field-oriented
     *     controls.
     * @return Wheel speeds [-1.0..1.0].
     */
    public MecanumDriveWheelSpeeds driveCartesianIK(
            double xSpeed, double ySpeed, double zRotation, double gyroAngle) {

        if( true ) {
            if (fieldRelative) {
                // Adjust for field orientation
                double radHeading = Math.toRadians(gyroAngle);
                double temp = xSpeed * Math.cos(radHeading) + ySpeed * Math.sin(radHeading);
                ySpeed = -xSpeed * Math.sin(radHeading) + ySpeed * Math.cos(radHeading);
                xSpeed = temp;
            }

            // Calculate wheel speeds based on the desired x, y, and rotation velocities
            return this.mecanumDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ySpeed, zRotation));
        }
        else {
            xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
            ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

            // Compensate for gyro angle.
            Translation2d input = new Translation2d(xSpeed, ySpeed).rotateBy(new Rotation2d(gyroAngle).unaryMinus());

            double[] wheelSpeeds = new double[4];
            wheelSpeeds[MotorType.kFrontLeft.value] = input.getX() + input.getY() + zRotation;
            wheelSpeeds[MotorType.kFrontRight.value] = input.getX() - input.getY() - zRotation;
            wheelSpeeds[MotorType.kRearLeft.value] = input.getX() - input.getY() + zRotation;
            wheelSpeeds[MotorType.kRearRight.value] = input.getX() + input.getY() - zRotation;

            normalize(wheelSpeeds);

            return new MecanumDriveWheelSpeeds(
                    wheelSpeeds[MotorType.kFrontLeft.value],
                    wheelSpeeds[MotorType.kFrontRight.value],
                    wheelSpeeds[MotorType.kRearLeft.value],
                    wheelSpeeds[MotorType.kRearRight.value]);
        }
    }

    /**
     * Sets the deadband applied to the drive inputs (e.g., joystick values).
     *
     * <p>The default value is {@value #kDefaultDeadband}. Inputs smaller than the deadband are set to
     * 0.0 while inputs larger than the deadband are scaled from 0.0 to 1.0. See {@link
     * edu.wpi.first.math.MathUtil#applyDeadband}.
     *
     * @param deadband The deadband to set.
     */
    public void setDeadband(double deadband) {
        m_deadband = deadband;
    }

    /**
     * Configure the scaling factor for using drive methods with motor controllers in a mode other
     * than PercentVbus or to limit the maximum output.
     *
     * <p>The default value is {@value #kDefaultMaxOutput}.
     *
     * @param maxOutput Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(double maxOutput) {
        m_maxOutput = maxOutput;
    }


    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     *
     * @param wheelSpeeds List of wheel speeds to normalize.
     */
    protected static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return getCurrentEstimatedPose();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        mecanumPoseEstimator.resetPosition(
                getGyroRotation2d(),//m_gyro.getRotation2d(),
                getWheelDistance(),//getCurrentWheelDistances(),
                pose);
    }
    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed Speed of the robot in the x direction (forward/backwards).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot,
                      boolean isRobotTurnOnly, double curHeading
    ) {
        if(isRobotTurnOnly)
        {
            driveCartesian(0, 0, rot);
        }
        else if (fieldRelative)
        {
            driveCartesian(xSpeed, ySpeed, rot, curHeading);
        } else
        {
            driveCartesian(xSpeed, ySpeed, rot);
        }
    }
    private boolean fieldRelative = false;
    private double angleOfRobotAndField = 0;
    public void setFieledRelative(boolean isFieldRela )
    {
        fieldRelative = isFieldRela;
        if(fieldRelative)
        {
            angleOfRobotAndField = getCurrentAngleDegree();
        }
        else
        {
            angleOfRobotAndField = 0;
        }
    }

    public void adjustToHeading(double targetAutoHeading, double currentHeading) {
        headingTurnPID.setSetpoint(targetAutoHeading);

        // This method should be called repeatedly, such as in a periodic or execute method in your command
        //double currentHeading = getCurrentAngleDegree();  // This method must return the current heading normalized to -180 to 180 degrees

        // Calculate the output from the PID controller, which automatically adjusts for the shortest path due to continuous input
        double rotationSpeed = headingTurnPID.calculate(currentHeading);
        DashServer.AddData("turnClcuSpeedRaw", rotationSpeed);

        // Optionally, you can limit the rotation speed here if needed
        rotationSpeed = Math.max(-1, Math.min(1, rotationSpeed));

        DashServer.AddData("turnTgtHding", targetAutoHeading);
        DashServer.AddData("turnCurHding", currentHeading);
        DashServer.AddData("turnClcuSpeed", rotationSpeed);

        // Drive the robot with the calculated rotation speed; ensure no other drive commands interfere
        drive(0, 0, rotationSpeed, true, currentHeading);  // Assuming drive method takes xSpeed, ySpeed, rotationSpeed, fieldOriented
    }

    public void stop()
    {
        frontLeft.set( 0);
        frontRight.set( 0);
        backLeft.set( 0);
        backRight.set( 0);
        frontLeft.motor.setPower(0);
        frontRight.motor.setPower(0);
        backLeft.motor.setPower(0);
        backRight.motor.setPower(0);
    }

    public double getCurrentAngleDegree()
    {
        return currentHeadingPi2NPi;//getGyroRotation2d().getDegrees(); gyroEx.getHeading().getDegrees()
    }

    public boolean getIsFieldRelative()
    {
        return fieldRelative;
    }

    public void resetHeading2Zero()
    {
        // so robot heading matchs the field forward direction, which is 0
        angleOfRobotAndField = 0;
        gyroEx.reset();
        currentHeadingPi2NPi = 0;
    }
}
