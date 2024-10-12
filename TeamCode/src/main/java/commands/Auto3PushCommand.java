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

                new commands.MecanumControllerCommand(
                        Config.Path.Trajectory_GotoA,
                        driveSubsystem::getCurrentEstimatedPose,
                        driveSubsystem.getKinematics(),
                        new edu.wpi.first.math.controller.PIDController(1,0.01,0.01),
                        new edu.wpi.first.math.controller.PIDController(2,0.01,0.01),
                        new edu.wpi.first.math.controller.ProfiledPIDController(
                                0.1,0,0.005,
                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                                        0.5,0.5)),
                        achievableMaxSpeed,
                        driveSubsystem::driveBySpeedEvent,
                        driveSubsystem).whenFinished(driveSubsystem::stopDrive)


























                // ,
                // new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)

//                ,
//                new commands.MecanumControllerCommand(
//                        Config.Path.Trajectory_PushA,
//                        driveSubsystem::getCurrentPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
//                                0.5,0.5)),
//                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
//                        driveSubsystem::driveBySpeed,
//                        telemetry,
//                        driveSubsystem ).whenFinished(driveSubsystem::stop)

                // ,
                // new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)

//                ,
//                new commands.MecanumControllerCommand(
//                        Config.Path.Trajectory_GotoB,
//                        driveSubsystem::getCurrentPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
//                                0.5,0.5)),
//                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
//                        driveSubsystem::driveBySpeed,
//                        telemetry,
//                        driveSubsystem).whenFinished(driveSubsystem::stop)

                // ,
                // new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)

//                ,
//                new commands.MecanumControllerCommand(
//                        Config.Path.Trajectory_PushB,
//                        driveSubsystem::getCurrentPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
//                                0.5,0.5)),
//                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
//                        driveSubsystem::driveBySpeed,
//                        telemetry,
//                        driveSubsystem ).whenFinished(driveSubsystem::stop)

                // ,
                // new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)

//                ,
//                new commands.MecanumControllerCommand(
//                        Config.Path.Trajectory_GotoC,
//                        driveSubsystem::getCurrentPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
//                                0.5,0.5)),
//                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
//                        driveSubsystem::driveBySpeed,
//                        telemetry,
//                        driveSubsystem).whenFinished(driveSubsystem::stop)
                        
                // ,
                // new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)

//                ,
//                new commands.MecanumControllerCommand(
//                        Config.Path.Trajectory_PushC,
//                        driveSubsystem::getCurrentPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.PIDController(1,0,0.01),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
//                                0.5,0.5)),
//                        ACHIEVABLE_MAX_DISTANCE_PER_SECOND,
//                        driveSubsystem::driveBySpeed,
//                        telemetry,
//                        driveSubsystem ).whenFinished(driveSubsystem::stop)

                // ,
                // new WaitCommand(2000).whenFinished(driveSubsystem::enableDrive)
        );
    }
}