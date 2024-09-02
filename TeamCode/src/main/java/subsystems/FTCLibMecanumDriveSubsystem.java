package subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import Config.DriveConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FTCLibMecanumDriveSubsystem extends SubsystemBase
{
    MecanumDriveKinematics mecanumDriveKinematics;
    MecanumDriveOdometry mecanumDriveOdometry;
    Motor frontLeft, backLeft, frontRight,  backRight;
    Motor.Encoder frontLeft_encoder, backLeft_encoder, frontRight_encoder, backRight_encoder;
    GyroEx gyroEx;
    //Pose2d currentPose;
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();
    MecanumDrive mecanumDrive;
    private Telemetry telemetry;

    AprilTagProcessor apriltag;

    public FTCLibMecanumDriveSubsystem(
			Motor frontLeft,
            Motor backLeft,
            Motor frontRight,
            Motor backRight,
            MecanumDriveKinematics kinematics,
            GyroEx gyro,
            AprilTagProcessor apriltag,
            Pose2d initialPose,
            Rotation2d initiaRotation,
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

        //currentPose = initialPose;


        mecanumDriveOdometry = new MecanumDriveOdometry (
                kinematics,
                initiaRotation,
                initialPose);

        //leftSpeed = gamepad::getLeftY;
        //rightSpeed = gamepad::getRightY;
        //mecanumDrive = new MecanumDrive(frontLeft,  backLeft, frontRight,  backRight);

        this.telemetry = telemetry;
    }

    public void reInitSystem()
    {
        mecanumDriveOdometry.resetPosition(new Pose2d(), new Rotation2d());
        gyroEx.init();
        gyroEx.reset();
    }

    MecanumDriveWheelSpeeds askedWheelSpeeds = new MecanumDriveWheelSpeeds();
    public void drive(MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds)
    {
        mecanumDriveWheelSpeeds.normalize(DriveConstants.MAX_VELOCITY);
        frontLeft.set( mecanumDriveWheelSpeeds.frontLeftMetersPerSecond);// DriveConstants.MAX_VELOCITY);
        frontRight.set( mecanumDriveWheelSpeeds.frontRightMetersPerSecond);// DriveConstants.MAX_VELOCITY);
        backLeft.set( mecanumDriveWheelSpeeds.rearLeftMetersPerSecond);// DriveConstants.MAX_VELOCITY);
        backRight.set( mecanumDriveWheelSpeeds.rearRightMetersPerSecond);// DriveConstants.MAX_VELOCITY);
        askedWheelSpeeds.rearRightMetersPerSecond = mecanumDriveWheelSpeeds.rearRightMetersPerSecond;
        askedWheelSpeeds.rearLeftMetersPerSecond = mecanumDriveWheelSpeeds.rearLeftMetersPerSecond;
        askedWheelSpeeds.frontLeftMetersPerSecond = mecanumDriveWheelSpeeds.frontLeftMetersPerSecond;
        askedWheelSpeeds.frontRightMetersPerSecond = mecanumDriveWheelSpeeds.frontRightMetersPerSecond;
    }

    @Override
    public void periodic() {
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        if(isSteopped)
        {
            telemetry.addLine("STOPPED!! /n");
        }
        else {
            wheelSpeeds.frontLeftMetersPerSecond = frontLeft_encoder.getRate();
            wheelSpeeds.frontRightMetersPerSecond = frontRight_encoder.getRate();
            wheelSpeeds.rearLeftMetersPerSecond = backLeft_encoder.getRate();
            wheelSpeeds.rearRightMetersPerSecond = backRight_encoder.getRate();

            // Update the pose
            mecanumDriveOdometry.updateWithTime(
                    System.currentTimeMillis() / 1000.0,
                    gyroEx.getRotation2d(), wheelSpeeds);

            // Convert to chassis speeds
            ChassisSpeeds chassisSpeeds =
                    mecanumDriveKinematics.toChassisSpeeds(wheelSpeeds);
        }

        telemetry.addData("Robot Estimator Position: ", mecanumDriveOdometry.getPoseMeters());
        telemetry.addLine();
        //telemetry.addData("Robot Rotations: ", gyroEx.getAngles()[1]);
        //telemetry.addData("Robot Speed: ", chassisSpeeds);
        telemetry.addData("Asked Wheels Speed: ", askedWheelSpeeds);
        telemetry.addLine();
        telemetry.addData("Measured Wheels Speed: ", wheelSpeeds);
        //telemetry.addData("Robot AbsoluteHeading: ", gyroEx.getAbsoluteHeading());
        //telemetryAprilTag();
        telemetry.update();

//        try {
//            sleep(5);
//        }catch (Exception e){
//            telemetry.addLine("Error!!!!!!!!!!!!!!");
//            telemetry.update();
//        }
    }

    boolean isSteopped = false;
    public void stop()
    {
        frontLeft.set( 0);// DriveConstants.MAX_VELOCITY);
        frontRight.set( 0);// DriveConstants.MAX_VELOCITY);
        backLeft.set( 0);// DriveConstants.MAX_VELOCITY);
        backRight.set( 0);// DriveConstants.MAX_VELOCITY);
        frontLeft.motor.setPower(0);
        frontRight.motor.setPower(0);
        backLeft.motor.setPower(0);
        backRight.motor.setPower(0);

//        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        isSteopped = true;
    }

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds()
    {
        return wheelSpeeds;
    }

    public Pose2d getCurrentPose()
    {
        return mecanumDriveOdometry.getPoseMeters();
    }

    public MecanumDriveKinematics getKinematics()
    {
        return mecanumDriveKinematics;
    }



    private void telemetryAprilTag() {

        ArrayList<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = apriltag.getDetections();
        //getFreshDetections()
        //getPerTagAvgPoseSolveTime() ms

        telemetry.addData("# AprilTags Detected: ", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Pose3d tagFieldPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, Math.PI));
                Transform3d tagToCamera = new Transform3d(new Translation3d(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z),
                        new Rotation3d(Units.degreesToRadians(detection.ftcPose.yaw),
                                Units.degreesToRadians(detection.ftcPose.pitch),
                                Units.degreesToRadians(detection.ftcPose.roll)))
                        .inverse();
                Pose3d cameraFieldPose = tagFieldPose.transformBy(tagToCamera);

                telemetry.addLine(String.format("==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (meter)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("BOT %6.1f %6.1f %6.1f  (xyz)", cameraFieldPose.getX(), cameraFieldPose.getY(), cameraFieldPose.getZ()));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }
}
