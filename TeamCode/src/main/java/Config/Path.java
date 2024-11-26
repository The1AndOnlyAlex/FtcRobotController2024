package Config;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;

public final class Path {

    public Path(){}

    private static TrajectoryConfig configWPI =
        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI);

    public static Trajectory exampleTrajectoryWPI_A =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        new edu.wpi.first.math.geometry.Pose2d(0, 0,
            new edu.wpi.first.math.geometry.Rotation2d(0)),
        Arrays.asList(
            new edu.wpi.first.math.geometry.Translation2d(0.5, 0),//1),
            new edu.wpi.first.math.geometry.Translation2d(1.0, 0)//-1)
        ),
        new edu.wpi.first.math.geometry.Pose2d(1.7, 0,
            new edu.wpi.first.math.geometry.Rotation2d(0)),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory exampleTrajectoryWPI_B =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(1.5, 0,
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(1.5, -0.6,
                new edu.wpi.first.math.geometry.Rotation2d(0))),

        configWPI
    );


    public static Trajectory exampleTrajectoryWPI_C =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(1.5, -0.6,
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(1.5, 0,
                new edu.wpi.first.math.geometry.Rotation2d(0))),

        configWPI
    );


    private static TrajectoryConfig configWPI_Revs =
        new edu.wpi.first.math.trajectory.TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION,
            true)
            .setKinematics(DriveConstants.kinematicsWPI);

    public static Trajectory exampleTrajectoryWPI_D =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(1.5, 0,
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(0, 0,
                new edu.wpi.first.math.geometry.Rotation2d(0))),
        configWPI_Revs
    );

    public static Trajectory exampleTrajectoryWPI_E =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
        new edu.wpi.first.math.geometry.Pose2d(1.7, 0,
            new edu.wpi.first.math.geometry.Rotation2d(0)),
        new edu.wpi.first.math.geometry.Pose2d(1.7, -0.8,
            new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(-90)))
        ),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory exampleTrajectoryWPI_F =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        new edu.wpi.first.math.geometry.Pose2d(1.4, -0.7,
            new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
        Arrays.asList(
            new edu.wpi.first.math.geometry.Translation2d(0.9, 0.0),//1),
            new edu.wpi.first.math.geometry.Translation2d(0.6, 0)//-1)
        ),
        new edu.wpi.first.math.geometry.Pose2d(0, 0,
            new edu.wpi.first.math.geometry.Rotation2d(0)),

        configWPI_Revs
    );

    public static Trajectory exampleTrajectoryWPI_G =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
        new edu.wpi.first.math.geometry.Pose2d(0, 0.1,
            new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
        new edu.wpi.first.math.geometry.Pose2d(-0.55, 0.1,
            new edu.wpi.first.math.geometry.Rotation2d(0))
        ),

        configWPI_Revs
    );


    public static Trajectory Trajectory_GotoA =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        new edu.wpi.first.math.geometry.Pose2d(0, 0,
            new edu.wpi.first.math.geometry.Rotation2d(0)),
        Arrays.asList(
            new edu.wpi.first.math.geometry.Translation2d(0.62, -0.62),
            new edu.wpi.first.math.geometry.Translation2d(1.4, -0.62)
        ),
        new edu.wpi.first.math.geometry.Pose2d(1.31, -1.06,
            new edu.wpi.first.math.geometry.Rotation2d(0)),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory Trajectory_PushA =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(1.30, -1.05,
                new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
            new edu.wpi.first.math.geometry.Pose2d(0, -1.05,
                new edu.wpi.first.math.geometry.Rotation2d(0))
        ),

        new edu.wpi.first.math.trajectory.TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION,
            true)
            .setKinematics(DriveConstants.kinematicsWPI)
    );


    public static Trajectory Trajectory_GotoB =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        new edu.wpi.first.math.geometry.Pose2d(0, -0.85,
            new edu.wpi.first.math.geometry.Rotation2d(0)),
        Arrays.asList(
            new edu.wpi.first.math.geometry.Translation2d(1.4, -0.75),
            new edu.wpi.first.math.geometry.Translation2d(1.4, -0.85)
        ),
        new edu.wpi.first.math.geometry.Pose2d(1.4, -1.05,
            new edu.wpi.first.math.geometry.Rotation2d(0)),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory Trajectory_PushB =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(1.4, -1.05,
                new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
            new edu.wpi.first.math.geometry.Pose2d(0, -1.05,
                new edu.wpi.first.math.geometry.Rotation2d(0))
        ),

        new edu.wpi.first.math.trajectory.TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION,
            true)
            .setKinematics(DriveConstants.kinematicsWPI)
    );


    public static Trajectory Trajectory_GotoC =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        new edu.wpi.first.math.geometry.Pose2d(0, -1.05,
            new edu.wpi.first.math.geometry.Rotation2d(0)),
        Arrays.asList(
            new edu.wpi.first.math.geometry.Translation2d(1.4, -0.95),
            new edu.wpi.first.math.geometry.Translation2d(1.4, -1.05)
        ),
        new edu.wpi.first.math.geometry.Pose2d(1.4, -1.25,
            new edu.wpi.first.math.geometry.Rotation2d(0)),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory Trajectory_PushC =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(1.4, -1.25,
                new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
            new edu.wpi.first.math.geometry.Pose2d(0, -1.25,
                new edu.wpi.first.math.geometry.Rotation2d(0))
        ),

        new edu.wpi.first.math.trajectory.TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION,
            true)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory Trajectory_GotoA1 =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
        new edu.wpi.first.math.geometry.Pose2d(0, 0,
            new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
        new edu.wpi.first.math.geometry.Pose2d(0.62, -0.62,
            new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0)))
        ),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory Trajectory_GotoA2 =
        edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(0.62, -0.62,
                new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),

            new edu.wpi.first.math.geometry.Pose2d(1.34, -0.62,
                new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0)))
        ),

        new TrajectoryConfig(
            DriveConstants.TRAJECTORY_MAX_VELOCITY,
            DriveConstants.MAX_ACCELERATION)
            .setKinematics(DriveConstants.kinematicsWPI)
    );
    public static Trajectory Trajectory_GotoA3 =
            edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new edu.wpi.first.math.geometry.Pose2d(1.34, -0.62,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
                new edu.wpi.first.math.geometry.Pose2d(1.33, -1.24, //1.31, -1.06
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0)))
            ),

            new edu.wpi.first.math.trajectory.TrajectoryConfig(
                DriveConstants.TRAJECTORY_MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION,
                true)
                .setKinematics(DriveConstants.kinematicsWPI)
    );


    public static Trajectory Trajectory_GotoA4 =
            edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new edu.wpi.first.math.geometry.Pose2d(1.33, -1.24,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
                new edu.wpi.first.math.geometry.Pose2d(1.34, -0.62,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0)))
            ),

            new edu.wpi.first.math.trajectory.TrajectoryConfig(
                DriveConstants.TRAJECTORY_MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION
                )
                .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory Trajectory_GotoA5 =
            edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new edu.wpi.first.math.geometry.Pose2d(1.34, -0.62,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),

                new edu.wpi.first.math.geometry.Pose2d(0.62, -0.62,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0)))
            ),

            new TrajectoryConfig(
                DriveConstants.TRAJECTORY_MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION,
                true)
                .setKinematics(DriveConstants.kinematicsWPI)
    );
    public static Trajectory Trajectory_GotoA6 =
            edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new edu.wpi.first.math.geometry.Pose2d(0.62, -0.62,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
                new edu.wpi.first.math.geometry.Pose2d(0,0,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0)))
            ),

            new TrajectoryConfig(
                DriveConstants.TRAJECTORY_MAX_VELOCITY,
                DriveConstants.MAX_ACCELERATION,
                true)
                .setKinematics(DriveConstants.kinematicsWPI)
    );
}
