package commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;
import util.RobotDataServer;

public class Auto3PushCommand extends SequentialCommandGroup 
{
    public Auto3PushCommand(
            MecanumDriveSubsystem driveSubsystem,
            IntakeSubsystem gripSubsystem,
            double achievableMaxSpeed//,
            //RobotDataServer dataServer
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
                        //dataServer,
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
                        //dataServer,
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
                        //dataServer,
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
                        //dataServer,
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
                        //dataServer,
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
                    //dataServer,
                    driveSubsystem).whenFinished(driveSubsystem::stopDrive)
//
//                ,
//                new commands.MecanumControllerCommand(
//                        Config.Path.Trajectory_PushA,
//                        driveSubsystem::getCurrentEstimatedPose,
//                        driveSubsystem.getKinematics(),
//                        new edu.wpi.first.math.controller.PIDController(0.1,0,0.01),
//                        new edu.wpi.first.math.controller.PIDController(0.1,0,0.01),
//                        new edu.wpi.first.math.controller.ProfiledPIDController(0.5,0,0,
//                                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
//                                0.5,0.5)),
//                        achievableMaxSpeed,
//                        driveSubsystem::driveBySpeedEvent,
//                        //dataServer,
//                        driveSubsystem).whenFinished(driveSubsystem::stopDrive)

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