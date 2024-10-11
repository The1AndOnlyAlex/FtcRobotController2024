//package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.function.Consumer;

import Config.ApriltagsFieldData;
import Config.DriveConstants;
import commands.AutoSequentialCommand;

import subsystems.FTCLibMecanumDriveSubsystem;
import subsystems.IntakeSubsystem;

@Disabled
@Autonomous
public class AutonomousFTCLibMecanumDrive extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    GyroEx gyro;
    private FTCLibMecanumDriveSubsystem ftclibMecanumDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private AprilTagProcessor aprilTag;
    private void initAprilTag() 
    {
        AprilTagLibrary apriltagLib;
        VisionPortal visionPortal;
        WebcamName apriltagCam;

        apriltagCam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the AprilTag processor.
        AprilTagLibrary.Builder libBuilder = new AprilTagLibrary.Builder();
        libBuilder.addTag(ApriltagsFieldData.tag_42);
        libBuilder.addTag(ApriltagsFieldData.tag_2);

        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(libBuilder.build());////AprilTagGameDatabase.getCurrentGameTagLibrary());
        //aprilTag =  new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

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
        aprilTag = myAprilTagProcessorBuilder.build();



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
        builder.setCameraResolution(new Size(640, 480)); //1280, 720));// fps 4
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);
        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);//YUY2);
        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

        //sleep(20);

    }   // end method initAprilTag()

    @Override
    public void initialize()
    {
        initAprilTag();

        frontLeft = new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);//RPM_435
        frontRight = new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);//RPM_312
        backLeft = new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        //frontLeft.setBuffer(0.7);
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

        frontLeft.setVeloCoefficients(1, 0, 0);
        frontLeft.setFeedforwardCoefficients(0, 0.1);
        frontRight.setVeloCoefficients(1, 0, 0);
        frontRight.setFeedforwardCoefficients(0, 0.1);
        backLeft.setVeloCoefficients(1, 0, 0);
        backLeft.setFeedforwardCoefficients(0, 0.1);
        backRight.setVeloCoefficients(1, 0, 0);
        backRight.setFeedforwardCoefficients(0, 0.1);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        frontLeft.encoder.reset();
        frontRight.encoder.reset();
        backLeft.encoder.reset();
        backRight.encoder.reset();
        frontLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        frontRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backLeft.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        backRight.encoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        double ACHIEVABLE_MAX_TICKS_PER_SECOND = frontLeft.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        double ACHIEVABLE_MAX_DISTANCE_PER_SECOND = ACHIEVABLE_MAX_TICKS_PER_SECOND * DriveConstants.DISTANCE_PER_PULSE;
        // about 1.567 m/s

        backRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro = new GyroEx() {
            IMU imu = hardwareMap.get(IMU.class, "imu");
            RevIMU a;
            @Override
            public void init() {
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

                // Now initialize the IMU with this mounting orientation
                // Note: if you choose two conflicting directions, this initialization will cause a code exception.
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
            public Rotation2d getRotation2d() {
//                imu.getRobotOrientationAsQuaternion()
                return new Rotation2d(getHeading());
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

        MecanumDriveKinematics kinematics =
                new MecanumDriveKinematics (
                        new Translation2d(0.2, 0.21),
                        new Translation2d(0.2, -0.21),
                        new Translation2d(-0.2, 0.21),
                        new Translation2d(-0.2, -0.21)
        );

        ftclibMecanumDriveSubsystem = new FTCLibMecanumDriveSubsystem(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                kinematics,
                gyro,
                aprilTag,
                new Pose2d(), // make sure it's initial pose
                new Rotation2d(), // make sure it's initial rotation
                new GamepadEx(gamepad1),
                telemetry
        );

        intakeSubsystem = new IntakeSubsystem(telemetry);


        TrajectoryConfig configWPI =
                new TrajectoryConfig(
                    DriveConstants.TRAJECTORY_MAX_VELOCITY,
                    DriveConstants.MAX_ACCELERATION)
                .setKinematics(kinematics);

        Trajectory exampleTrajectoryWPI_A = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0,
                        new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                Arrays.asList(
                        new Translation2d(0.5, 0),//1),
                        new Translation2d(1.0, 0)//-1)
                ),
                // End 5 meters straight ahead of where we started, facing forward
                new Pose2d(1.5, 0,
                        new Rotation2d(0)),
                // Pass config
                configWPI
        );

        Trajectory exampleTrajectoryWPI_B = TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                // Start at the origin facing the +X direction
                new Pose2d(1.5, 0,
                        new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
//                Arrays.asList(
//                        new edu.wpi.first.math.geometry.Translation2d(1.5, -1),//1),
//                        new edu.wpi.first.math.geometry.Translation2d(1.5, -2)//-1)
//                ),
                // End 5 meters straight ahead of where we started, facing forward
                new Pose2d(1.5, -0.5,
                        new Rotation2d(0))),
                // Pass config
                configWPI
        );

//        ftclibMecanumDriveSubsystem.setDefaultCommand(
//        new ApriltagCommand(wpiMecanumDriveSubsystem, aprilTag, telemetry));

        // update telemetry every loop
        //schedule(new RunCommand(telemetry::update));

        schedule(new MecanumControllerCommand(
                exampleTrajectoryWPI_A,
                ftclibMecanumDriveSubsystem::getCurrentPose,
                kinematics,
                new PController(1,0,0),
                new PIDController(1,0,0),
                new ProfiledPIDController(0.5,0,0,
                        new TrapezoidProfile.Constraints(0.5,0.5)),
                ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
                ftclibMecanumDriveSubsystem::drive).whenFinished(ftclibMecanumDriveSubsystem::stop));
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        commands.IntakeGrabCommand a = new commands.IntakeGrabCommand(intakeSubsystem, telemetry);

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
//            telemetry.addLine("Doing nothing");
//            telemetry.update();

            sleep(10);
        }
        reset();
    }

    public void letswait()
    {
        sleep(20);
    }
}
