package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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

public class WpiMecanumDriveSubsystem extends SubsystemBase
{
    MecanumDriveKinematics mecanumDriveKinematics;
    MecanumDrivePoseEstimator mecanumPoseEstimator;
    Motor frontLeft, backLeft, frontRight,  backRight;
    Motor.Encoder frontLeft_encoder, backLeft_encoder, frontRight_encoder, backRight_encoder;
    GyroEx gyroEx;
    MecanumDriveWheelSpeeds wheelSpeeds;
    MecanumDrive mecanumDrive;
    private Telemetry telemetry;
    AprilTagProcessor apriltag;

    Pose2d currentPose;

    public WpiMecanumDriveSubsystem(
            Motor frontLeft,
            Motor frontRight,
            Motor backLeft,
            Motor backRight,
            MecanumDriveKinematics kinematics,
            GyroEx gyro,
            AprilTagProcessor apriltag,
            Pose2d initialPose,
            GamepadEx gamepad,
            Telemetry telemetry)
    {
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;

        this.frontLeft_encoder = frontLeft.encoder;
        this.backLeft_encoder = backLeft.encoder;
        this.frontRight_encoder = frontRight.encoder;
        this.backRight_encoder = backRight.encoder;

        mecanumDriveKinematics = kinematics;

        gyroEx = gyro;

        this.apriltag = apriltag;

        currentPose = initialPose;

        //leftSpeed = gamepad::getLeftY;
        //rightSpeed = gamepad::getRightY;

        mecanumPoseEstimator = new MecanumDrivePoseEstimator(
                mecanumDriveKinematics,
                getGyroRotation2d(),
                getWheelDistance(),//getWheelPositions(),
                initialPose,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // JD TODO: Set pose estimator weights
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        mecanumDrive = new MecanumDrive(frontLeft,  backLeft, frontRight,  backRight);

        this.telemetry = telemetry;
    }

    private MecanumDriveWheelPositions getWheelDistance() {
        return new MecanumDriveWheelPositions(
                frontLeft_encoder.getDistance(),
                backLeft_encoder.getDistance(),
                frontRight_encoder.getDistance(),
                backRight_encoder.getDistance());
    }

//    private MecanumDriveWheelPositions getWheelPositions() {
//        return new MecanumDriveWheelPositions(
//                frontLeft_encoder.getPosition(),
//                backLeft_encoder.getPosition(),
//                frontRight_encoder.getPosition(),
//                backRight_encoder.getPosition());
//    }

	MecanumDriveWheelSpeeds askedWheelSpeeds = new MecanumDriveWheelSpeeds();
    public void driveBySpeed(MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds)
    {
		mecanumDriveWheelSpeeds.desaturate(DriveConstants.MAX_VELOCITY);
        frontLeft.set( mecanumDriveWheelSpeeds.frontLeftMetersPerSecond);/// DriveConstants.MAX_VELOCITY);
        frontRight.set( mecanumDriveWheelSpeeds.frontRightMetersPerSecond);/// DriveConstants.MAX_VELOCITY);
        backLeft.set( mecanumDriveWheelSpeeds.rearLeftMetersPerSecond);/// DriveConstants.MAX_VELOCITY);
        backRight.set( mecanumDriveWheelSpeeds.rearRightMetersPerSecond);/// DriveConstants.MAX_VELOCITY);

        askedWheelSpeeds.rearRightMetersPerSecond = mecanumDriveWheelSpeeds.rearRightMetersPerSecond;
        askedWheelSpeeds.rearLeftMetersPerSecond = mecanumDriveWheelSpeeds.rearLeftMetersPerSecond;
        askedWheelSpeeds.frontLeftMetersPerSecond = mecanumDriveWheelSpeeds.frontLeftMetersPerSecond;
        askedWheelSpeeds.frontRightMetersPerSecond = mecanumDriveWheelSpeeds.frontRightMetersPerSecond;
    }

//    public void driveByVoltage(MecanumDriveMotorVoltages mecanumDriveMotorVoltages)
//    {
//        frontLeft.set( mecanumDriveMotorVoltages.frontLeftVoltage);
//        frontRight.set( mecanumDriveMotorVoltages.frontRightVoltage);
//        backLeft.set( mecanumDriveMotorVoltages.rearLeftVoltage);
//        backRight.set( mecanumDriveMotorVoltages.rearRightVoltage);
//    }

    boolean enableVisionPE = false;
//    public double getGyroHeading()
//    {
//        return (gyroEx.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));//gyroEx.getHeading());//Rotation2d.fromDegrees
//        //return gyro.getRotation2d();
//    }

    public Rotation2d getGyroRotation2d()
    {
        return new Rotation2d(gyroEx.getRotation2d().getRadians());
    }

    Pose2d VisionPose2dPre = new Pose2d();
    @Override
    public void periodic() {
//        isSteopped = true;
        if(isSteopped)
        {
            telemetry.addData("STOPPED?? ", isSteopped);
        }
        else {
            // Get my wheel speeds; assume .getRate() has been
            // set up to return velocity of the encoder
            // in meters per second.
            wheelSpeeds = new MecanumDriveWheelSpeeds
            (
                    frontLeft_encoder.getRate(), frontRight_encoder.getRate(),
                    backLeft_encoder.getRate(), backRight_encoder.getRate()
            );

            MecanumDriveWheelPositions mecanumDriveWheelPositions = new MecanumDriveWheelPositions(
                    frontLeft_encoder.getDistance(), frontRight_encoder.getDistance(),
                    backLeft_encoder.getDistance(), backRight_encoder.getDistance());


            // Update the pose
            currentPose = mecanumPoseEstimator.update(
                    getGyroRotation2d(), mecanumDriveWheelPositions);

        }

        if (enableVisionPE) {
            Pose2d botPose1 = visionupdateOdometry();
            //visionupdateOdometry(camera1);
            //visionupdateOdometry(camera2);
            if(botPose1 != null) {
                telemetry.addData("Vision Pose: ", botPose1);
                VisionPose2dPre = botPose1;
            }
        }

        telemetry.addData("last Vision Pose: ", VisionPose2dPre);
        telemetry.addLine();
        telemetry.addData("Robot Estimator Position: ", currentPose);//mecanumPoseEstimator.getEstimatedPosition());
        telemetry.addLine();
        telemetry.addData("Asked Wheels Speed: ", askedWheelSpeeds);
        telemetry.addLine();
        telemetry.addData("Measured Wheels Speed: ", wheelSpeeds);
        //telemetry.addData("Robot Speed: ", mecanumDriveKinematics.toChassisSpeeds(wheelSpeeds);
        //telemetry.addData("Robot Rotation: ", gyroEx.());
        //telemetryAprilTag();
        telemetry.update();
    }
	public void enableDrive()
    {
        isSteopped = false;
    }
	static boolean isSteopped = false;
    public void stop()
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

    private Pose2d ProcessApriltagData()
    {
        Pose2d visionPose = null;

        List<AprilTagDetection> currentDetections = apriltag.getDetections();
        //getFreshDetections()
        //getPerTagAvgPoseSolveTime() ms

        telemetry.addData("# AprilTags Detected: ", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Pose3d tagFieldPose = new Pose3d(new Translation3d(0, 0, 0.07), new Rotation3d(0, 0, 0));
                Transform3d tagToCamera = new Transform3d(new Translation3d(detection.ftcPose.y, -1 * detection.ftcPose.x, detection.ftcPose.z),
                        new Rotation3d(Units.degreesToRadians(detection.ftcPose.roll),
                                Units.degreesToRadians(detection.ftcPose.pitch),
                                Units.degreesToRadians(detection.ftcPose.yaw)));
                //.inverse();
                Pose3d cameraFieldPose = ComputerVisionUtil.objectToRobotPose(tagFieldPose, tagToCamera, DriveConstants.CamToRobot); //tagFieldPose.transformBy(tagToCamera);

                // error!
                telemetry.addLine(String.format("ComputerVisionUtil %6.3f %6.3f %6.1f  (xyr)", -1 * cameraFieldPose.getX(), -1 * cameraFieldPose.getY(),
                        -1 * detection.ftcPose.yaw));
            }
        }
        return visionPose;
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = apriltag.getDetections();
        //getFreshDetections()
        //getPerTagAvgPoseSolveTime() ms

        telemetry.addData("# AprilTags Detected: ", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Pose3d tagFieldPose = new Pose3d(new Translation3d(0, 0, 0.07), new Rotation3d(0, 0, 0));
                Transform3d tagToCamera = new Transform3d(new Translation3d(detection.ftcPose.y, -1*detection.ftcPose.x, detection.ftcPose.z),
                        new Rotation3d(Units.degreesToRadians(detection.ftcPose.roll),
                                Units.degreesToRadians(detection.ftcPose.pitch),
                                Units.degreesToRadians(detection.ftcPose.yaw)));
                        //.inverse();
                Pose3d cameraFieldPose = ComputerVisionUtil.objectToRobotPose(tagFieldPose, tagToCamera, DriveConstants.CamToRobot); //tagFieldPose.transformBy(tagToCamera);

                // error!
                telemetry.addLine(String.format("ComputerVisionUtil %6.3f %6.3f %6.1f  (xyr)", -1*cameraFieldPose.getX(), -1*cameraFieldPose.getY(),
                        -1*detection.ftcPose.yaw));

                telemetry.addLine(String.format("==== (ID %d) %s", detection.id, detection.metadata.name));
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
            } else {
                telemetry.addLine(String.format("==== (ID %d) Unknown", detection.id));
                //telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }

    private Pose2d visionupdateOdometry() {
        boolean doRejectUpdate = false;

        double xyStds = 0.5;
        double degStds = 999999;

        Pose2d botPose1 = null;//new Pose2d(-1,-1, new Rotation2d(0));
        boolean receivedValidData = false;
        double timeAcquisition = 0;

        List<AprilTagDetection> currentDetections = apriltag.getDetections();//getFreshDetections();//.getDetections();
        if(currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((detection.id == ApriltagsFieldData.tag_2.id)
//                            && (detection.ftcPose.z > -0.2)
//                            && (detection.ftcPose.z < 0.2)
//                            && (detection.ftcPose.range < 2.0)
                    ) {
                        Pose3d tagFieldPose = new Pose3d(new Translation3d(0, 0, 0.07), new Rotation3d(0, 0, 0));
                        Transform3d tagToCamera = new Transform3d(new Translation3d(detection.ftcPose.y, -1 * detection.ftcPose.x, detection.ftcPose.z),
                                new Rotation3d(Units.degreesToRadians(detection.ftcPose.roll),
                                        Units.degreesToRadians(detection.ftcPose.pitch),
                                        Units.degreesToRadians(detection.ftcPose.yaw)));
                        //.inverse();
                        Pose3d cameraFieldPose = ComputerVisionUtil.objectToRobotPose(tagFieldPose, tagToCamera, DriveConstants.CamToRobot); //tagFieldPose.transformBy(tagToCamera);

                        botPose1 = new Pose2d(-1 * cameraFieldPose.getX(), -1 * cameraFieldPose.getY(),
                                new Rotation2d(Units.degreesToRadians(-1 * detection.ftcPose.yaw)));

                        timeAcquisition = detection.frameAcquisitionNanoTime / 1e-9;
                        // trust it more <-> less
                        xyStds = 0.5;//1.0;
                        degStds = 6;//12;
                    } else {
                        doRejectUpdate = true;
                    }
                } else {
                    doRejectUpdate = true;
                }
            }
        }
        else {
            doRejectUpdate = true;
        }


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

        if(!doRejectUpdate)
        {
            mecanumPoseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

            //frc poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            if(botPose1 != null)
                mecanumPoseEstimator.addVisionMeasurement(
                    botPose1,//mt1.pose,
                    timeAcquisition);//mt1.timestampSeconds);
        }

        return botPose1;
    }

    public Pose2d getCurrentPose()
    {
        return mecanumPoseEstimator.getEstimatedPosition();
    }

    public MecanumDriveKinematics getKinematics()
    {
        return mecanumDriveKinematics;
    }
}
