package commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;

public class Auto3PushCommand extends SequentialCommandGroup 
{
    public Auto3PushCommand(
            MecanumDriveSubsystem driveSubsystem,
            IntakeSubsystem gripSubsystem,
            double achievableMaxSpeed
    )
    {
        addCommands(
            //new InstantCommand(driveSubsystem::enableDrive),
            new commands.MecanumControllerCommand(
                Config.Path.Trajectory_GotoA1,
                driveSubsystem::getCurrentEstimatedPose,
                driveSubsystem.getKinematics(),
                new edu.wpi.first.math.controller.PIDController(1,0,0.001),
                new edu.wpi.first.math.controller.PIDController(0.8,0,0.001),
                new edu.wpi.first.math.controller.ProfiledPIDController(
                        0.1,0,0.005,
                        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                0.5,0.5)),
                achievableMaxSpeed,
                driveSubsystem::driveBySpeedEvent,
                driveSubsystem)//.whenFinished(driveSubsystem::stopDrive)

             ,
            new commands.MecanumControllerCommand(
                Config.Path.Trajectory_GotoA2,
                driveSubsystem::getCurrentEstimatedPose,
                driveSubsystem.getKinematics(),
                new edu.wpi.first.math.controller.PIDController(0.05,0,0.001),
                new edu.wpi.first.math.controller.PIDController(0.05,0,0.001),
                new edu.wpi.first.math.controller.ProfiledPIDController(
                        0.1,0,0.005,
                        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                0.5,0.5)),
                achievableMaxSpeed,
                driveSubsystem::driveBySpeedEvent,
                driveSubsystem)//.whenFinished(driveSubsystem::stopDrive)

            ,
            new commands.MecanumControllerCommand(
                Config.Path.Trajectory_GotoA3,
                driveSubsystem::getCurrentEstimatedPose,
                driveSubsystem.getKinematics(),
                new edu.wpi.first.math.controller.PIDController(0.3,0,0.001),
                new edu.wpi.first.math.controller.PIDController(0.2,0,0.001),
                new edu.wpi.first.math.controller.ProfiledPIDController(
                        0.1,0,0.005,
                        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                0.5,0.5)),
                achievableMaxSpeed,
                driveSubsystem::driveBySpeedEvent,
                driveSubsystem).whenFinished(driveSubsystem::stopDrive)

            ,
             new WaitCommand(1000).whenFinished(driveSubsystem::enableDrive)

            ,
            new commands.MecanumControllerCommand(
                Config.Path.Trajectory_GotoA4,
                driveSubsystem::getCurrentEstimatedPose,
                driveSubsystem.getKinematics(),
                new edu.wpi.first.math.controller.PIDController(0.3,0,0.001),
                new edu.wpi.first.math.controller.PIDController(0.2,0,0.001),
                new edu.wpi.first.math.controller.ProfiledPIDController(
                        0.1,0,0.005,
                        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                0.5,0.5)),
                achievableMaxSpeed,
                driveSubsystem::driveBySpeedEvent,
                driveSubsystem)//.whenFinished(driveSubsystem::stopDrive)

            ,
            new commands.MecanumControllerCommand(
                Config.Path.Trajectory_GotoA5,
                driveSubsystem::getCurrentEstimatedPose,
                driveSubsystem.getKinematics(),
                new edu.wpi.first.math.controller.PIDController(0.05,0,0.001),
                new edu.wpi.first.math.controller.PIDController(0.05,0,0.001),
                new edu.wpi.first.math.controller.ProfiledPIDController(
                        0.1,0,0.005,
                        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                0.5,0.5)),
                achievableMaxSpeed,
                driveSubsystem::driveBySpeedEvent,
                driveSubsystem)//.whenFinished(driveSubsystem::stopDrive)

            ,
            new commands.MecanumControllerCommand(
                Config.Path.Trajectory_GotoA6,
                driveSubsystem::getCurrentEstimatedPose,
                driveSubsystem.getKinematics(),
                new edu.wpi.first.math.controller.PIDController(1,0,0.001),
                new edu.wpi.first.math.controller.PIDController(0.8,0,0.001),
                new edu.wpi.first.math.controller.ProfiledPIDController(
                        0.1,0,0.005,
                        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                0.5,0.5)),
                achievableMaxSpeed,
                driveSubsystem::driveBySpeedEvent,
                driveSubsystem).whenFinished(driveSubsystem::stopDrive)
        );
    }
}