package commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
//import com.arcrobotics.ftclib.trajectory.Trajectory;
//import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import subsystems.IntakeSubsystem;
//import subsystems.MecanumDriveSubsystem;
import subsystems.WpiMecanumDriveSubsystem;

public class AutoSequentialCommand extends SequentialCommandGroup {

    public AutoSequentialCommand(
            WpiMecanumDriveSubsystem driveSubsystem,
            IntakeSubsystem gripSubsystem,
            double ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
            Telemetry telemetry
    )
    {
        addCommands(

                new commands.MecanumControllerCommand(
                        Config.Path.exampleTrajectoryWPI_A,
                        driveSubsystem::getCurrentPose,
                        driveSubsystem.getKinematics(),
                        new edu.wpi.first.math.controller.PIDController(1,0,0),
                        new edu.wpi.first.math.controller.PIDController(1,0,0),
                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(0.5,0.5)),
                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND, // DriveConstants.TRAJECTORY_MAX_VELOCITY,
                        driveSubsystem::driveBySpeed,
                        telemetry,
                        driveSubsystem).whenFinished(driveSubsystem::stop)



//                new commands.MecanumControllerCommand(
//                        exampleTrajectory_A,
//                        driveSubsystem::getCurrentPose,
//                        new SimpleMotorFeedforward(0.01, 0.01, 0.01),
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(0.1,0,0),
//                        new edu.wpi.first.math.controller.PIDController(0.1,0,0),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.1,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(1.5,1.5)),
//                        false,
//                        DriveConstants.TRAJECTORY_MAX_VELOCITY,//.MAX_VELOCITY, // need to be higher than trajectory??
//                        new edu.wpi.first.math.controller.PIDController(0.01,0,0),// frontLeftController,
//                        new edu.wpi.first.math.controller.PIDController(0.01,0,0),// rearLeftController,
//                        new edu.wpi.first.math.controller.PIDController(0.01,0,0),// frontRightController,
//                        new edu.wpi.first.math.controller.PIDController(0.01,0,0),// rearRightController,
//                        driveSubsystem::getCurrentwheelSpeeds,
//                        driveSubsystem::driveByVoltage,
//                        telemetry,
//                        driveSubsystem)
//                ,
                //new IntakeGrabCommand(gripSubsystem,telemetry)
                ,
                new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)

                ,
                new commands.MecanumControllerCommand(
                        Config.Path.exampleTrajectoryWPI_E,
                        driveSubsystem::getCurrentPose,
                        driveSubsystem.getKinematics(),
                        new edu.wpi.first.math.controller.PIDController(0.01,0,0),
                        new edu.wpi.first.math.controller.PIDController(0.01,0,0),
                        new edu.wpi.first.math.controller.ProfiledPIDController(9.7,0.0,0.1,
                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(9.7,9.7)),
                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,//DriveConstants.TRAJECTORY_MAX_VELOCITY,//.MAX_VELOCITY, // need to be higher than trajectory??
                        driveSubsystem::driveBySpeed,
                        telemetry,
                        driveSubsystem ).whenFinished(driveSubsystem::stop)
                ,
                new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)
//                ,
//                new IntakeReleaseCommand(gripSubsystem)
                ,
                new commands.MecanumControllerCommand(
                        Config.Path.exampleTrajectoryWPI_F,
                        driveSubsystem::getCurrentPose,
                        driveSubsystem.getKinematics(),
                        new edu.wpi.first.math.controller.PIDController(.1,0,0),
                        new edu.wpi.first.math.controller.PIDController(.1,0,0),
                        new edu.wpi.first.math.controller.ProfiledPIDController(10,0,0,
                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(10,10)),
                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,//DriveConstants.TRAJECTORY_MAX_VELOCITY,//.MAX_VELOCITY, // need to be higher than trajectory??
                        driveSubsystem::driveBySpeed,
                        telemetry,
                        driveSubsystem )//.whenFinished(driveSubsystem::stop)

//                ,
//                new commands.MecanumControllerCommand(
//                        Path.exampleTrajectory_D,
//                        driveSubsystem::getCurrentPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(0.5,0.5)),
//                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,//DriveConstants.TRAJECTORY_MAX_VELOCITY,//.MAX_VELOCITY, // need to be higher than trajectory??
//                        driveSubsystem::driveBySpeed,
//                        telemetry,
//                        driveSubsystem ).whenFinished(driveSubsystem::stop)
                ,
                new commands.MecanumControllerCommand( // to fine tune for finish line
                        Config.Path.exampleTrajectoryWPI_G,
                        driveSubsystem::getCurrentPose,
                        driveSubsystem.getKinematics(),
                        new edu.wpi.first.math.controller.PIDController(0.5,0,0),
                        new edu.wpi.first.math.controller.PIDController(0.5,0,0),
                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(0.5,0.5)),
                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,//DriveConstants.TRAJECTORY_MAX_VELOCITY,//.MAX_VELOCITY, // need to be higher than trajectory??
                        driveSubsystem::driveBySpeed,
                        telemetry,
                        driveSubsystem ).whenFinished(driveSubsystem::stop)
        );
    }
}