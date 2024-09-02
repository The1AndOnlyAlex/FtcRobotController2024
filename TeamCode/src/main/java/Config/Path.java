package Config;

import java.util.Arrays;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;

public final class Path {
//        TrajectoryConfig config = new TrajectoryConfig(
//                DriveConstants.TRAJECTORY_MAX_VELOCITY,
//                DriveConstants.MAX_ACCELERATION)
//                .setKinematics(kinematics);
//
//        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//                // Start at the origin facing the +X direction
//                new Pose2d(0, 0, new Rotation2d(0)),
//                // Pass through these two interior waypoints, making an 's' curve path
//                Arrays.asList(new Translation2d(1, 1), new Translation2d(2, -1)),
//                // End 3 meters straight ahead of where we started, facing forward
//                new Pose2d(3, 0, new Rotation2d(0)),
//                // Pass config
//                config
//        );

    public Path(){}


    private static TrajectoryConfig configWPI =
            new TrajectoryConfig(
                    DriveConstants.TRAJECTORY_MAX_VELOCITY,
                    DriveConstants.MAX_ACCELERATION)
                    .setKinematics(DriveConstants.kinematicsWPI);

    public static Trajectory exampleTrajectoryWPI_A = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
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

    public static Trajectory exampleTrajectoryWPI_B = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                    new edu.wpi.first.math.geometry.Pose2d(1.5, 0,
                            new edu.wpi.first.math.geometry.Rotation2d(0)),
                    new edu.wpi.first.math.geometry.Pose2d(1.5, -0.6,
                            new edu.wpi.first.math.geometry.Rotation2d(0))),

            configWPI
    );


    public static Trajectory exampleTrajectoryWPI_C = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
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
//configWPI_Revs.setReversed(true);

    public static Trajectory exampleTrajectoryWPI_D = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                    new edu.wpi.first.math.geometry.Pose2d(1.5, 0,
                            new edu.wpi.first.math.geometry.Rotation2d(0)),
                    new edu.wpi.first.math.geometry.Pose2d(0, 0,
                            new edu.wpi.first.math.geometry.Rotation2d(0))),
            configWPI_Revs
    );

    public static Trajectory exampleTrajectoryWPI_E = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            new edu.wpi.first.math.geometry.Pose2d(1.7, 0,
                    new edu.wpi.first.math.geometry.Rotation2d(0)),
            Arrays.asList(
                    new edu.wpi.first.math.geometry.Translation2d(1.7, -0.3),//1),
                    new edu.wpi.first.math.geometry.Translation2d(1.7, -0.6)//-1)
            ),
            new edu.wpi.first.math.geometry.Pose2d(1.7, -0.8,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(-90))),

            new TrajectoryConfig(
                    DriveConstants.TRAJECTORY_MAX_VELOCITY,
                    DriveConstants.MAX_ACCELERATION)
                    .setKinematics(DriveConstants.kinematicsWPI)
    );

    public static Trajectory exampleTrajectoryWPI_F = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
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

    public static Trajectory exampleTrajectoryWPI_G = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
            new edu.wpi.first.math.geometry.Pose2d(0, 0.1,
                    new edu.wpi.first.math.geometry.Rotation2d(Units.degreesToRadians(0))),
//            Arrays.asList(
//                    new edu.wpi.first.math.geometry.Translation2d(1.5, 0),//1),
//                    new edu.wpi.first.math.geometry.Translation2d(1.0, 0)//-1)
//            ),
            new edu.wpi.first.math.geometry.Pose2d(-0.5, 0.1,
                    new edu.wpi.first.math.geometry.Rotation2d(0))
            ),

            configWPI_Revs
    );

}
