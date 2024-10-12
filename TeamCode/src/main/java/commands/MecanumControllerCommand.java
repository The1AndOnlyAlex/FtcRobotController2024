// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import util.DashServer;
import java.util.function.Consumer;
import java.util.function.Supplier;


/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a mecanum drive.
 *
 * <p>The command handles trajectory-following, Velocity PID calculations, and feedforwards
 * internally. This is intended to be a more-or-less "complete solution" that can be used by teams
 * without a great deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the PID controllers.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class MecanumControllerCommand extends CommandBase {
  private Timer m_timer;
  private final boolean m_usePID;
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SimpleMotorFeedforward m_feedforward;
  private final MecanumDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final double m_maxWheelVelocityMetersPerSecond;
  private final PIDController m_frontLeftController;
  private final PIDController m_rearLeftController;
  private final PIDController m_frontRightController;
  private final PIDController m_rearRightController;
  private final Supplier<MecanumDriveWheelSpeeds> m_currentWheelSpeeds;
  private final Consumer<MecanumDriveMotorVoltages> m_outputDriveVoltages;
  private final Consumer<MecanumDriveWheelSpeeds> m_outputWheelSpeeds;
  private  double
  //final MutableMeasure<Velocity<Distance>>
          m_prevFrontLeftSpeedSetpoint = 0;
      //MutableMeasure.zero(MetersPerSecond);
  private  double
      //final MutableMeasure<Velocity<Distance>>
              m_prevRearLeftSpeedSetpoint = 0;
      //MutableMeasure.zero(MetersPerSecond);
  private  double
  //final MutableMeasure<Velocity<Distance>>
      m_prevFrontRightSpeedSetpoint = 0;
      //MutableMeasure.zero(MetersPerSecond);
  private  double
//  final MutableMeasure<Velocity<Distance>>
          m_prevRearRightSpeedSetpoint = 0;
//      MutableMeasure.zero(MetersPerSecond);
  private  double
//  final MutableMeasure<Velocity<Distance>>
          m_frontLeftSpeedSetpoint = 0;
//      MutableMeasure.zero(MetersPerSecond);
  private  double
//  final MutableMeasure<Velocity<Distance>>
          m_rearLeftSpeedSetpoint = 0;
//      MutableMeasure.zero(MetersPerSecond);
  private  double
//  final MutableMeasure<Velocity<Distance>>
          m_frontRightSpeedSetpoint = 0;
//      MutableMeasure.zero(MetersPerSecond);
  private  double
//  final MutableMeasure<Velocity<Distance>>
          m_rearRightSpeedSetpoint = 0;
//      MutableMeasure.zero(MetersPerSecond);

  /**
   * Constructs a new MecanumControllerCommand that when executed will follow the provided
   * trajectory. PID control and feedforward are handled internally. Outputs are scaled from -12 to
   * 12 as a voltage output to the motor.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param feedforward The feedforward to use for the drivetrain.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param frontLeftController The front left wheel velocity PID.
   * @param rearLeftController The rear left wheel velocity PID.
   * @param frontRightController The front right wheel velocity PID.
   * @param rearRightController The rear right wheel velocity PID.
   * @param currentWheelSpeeds A MecanumDriveWheelSpeeds object containing the current wheel speeds.
   * @param outputDriveVoltages A MecanumDriveMotorVoltages object containing the output motor
   *     voltages.
   * @param requirements The subsystems to require.
   */
//  @SuppressWarnings("this-escape")
  public MecanumControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SimpleMotorFeedforward feedforward,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      boolean needNextSateRotation,//<Rotation2d> desiredRotation,
      double maxWheelVelocityMetersPerSecond,
      PIDController frontLeftController,
      PIDController rearLeftController,
      PIDController frontRightController,
      PIDController rearRightController,
      Supplier<MecanumDriveWheelSpeeds> currentWheelSpeeds,
      Consumer<MecanumDriveMotorVoltages> outputDriveVoltages,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "MecanumControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "MecanumControllerCommand");
    m_feedforward = requireNonNullParam(feedforward, "feedforward", "MecanumControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "MecanumControllerCommand");

    m_controller =
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "MecanumControllerCommand"),
            requireNonNullParam(yController, "yController", "MecanumControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "MecanumControllerCommand"));

    if (!needNextSateRotation)
      m_desiredRotation = ()->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
    else // TBD need to work on it. JD
      m_desiredRotation = ()->
              trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
//    m_desiredRotation =
//        requireNonNullParam(desiredRotation, "desiredRotation", "MecanumControllerCommand");

    m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;

    m_frontLeftController =
        requireNonNullParam(frontLeftController, "frontLeftController", "MecanumControllerCommand");
    m_rearLeftController =
        requireNonNullParam(rearLeftController, "rearLeftController", "MecanumControllerCommand");
    m_frontRightController =
        requireNonNullParam(
            frontRightController, "frontRightController", "MecanumControllerCommand");
    m_rearRightController =
        requireNonNullParam(rearRightController, "rearRightController", "MecanumControllerCommand");

    m_currentWheelSpeeds =
        requireNonNullParam(currentWheelSpeeds, "currentWheelSpeeds", "MecanumControllerCommand");

    m_outputDriveVoltages =
        requireNonNullParam(outputDriveVoltages, "outputDriveVoltages", "MecanumControllerCommand");

    m_outputWheelSpeeds = null;

    m_usePID = true;

    addRequirements(requirements);
  }

  /**
   * Constructs a new MecanumControllerCommand that when executed will follow the provided
   * trajectory. PID control and feedforward are handled internally. Outputs are scaled from -12 to
   * 12 as a voltage output to the motor.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param feedforward The feedforward to use for the drivetrain.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param frontLeftController The front left wheel velocity PID.
   * @param rearLeftController The rear left wheel velocity PID.
   * @param frontRightController The front right wheel velocity PID.
   * @param rearRightController The rear right wheel velocity PID.
   * @param currentWheelSpeeds A MecanumDriveWheelSpeeds object containing the current wheel speeds.
   * @param outputDriveVoltages A MecanumDriveMotorVoltages object containing the output motor
   *     voltages.
   * @param requirements The subsystems to require.
   */
//  public MecanumControllerCommand(
//      Trajectory trajectory,
//      Supplier<Pose2d> pose,
//      SimpleMotorFeedforward feedforward,
//      MecanumDriveKinematics kinematics,
//      PIDController xController,
//      PIDController yController,
//      ProfiledPIDController thetaController,
//      double maxWheelVelocityMetersPerSecond,
//      PIDController frontLeftController,
//      PIDController rearLeftController,
//      PIDController frontRightController,
//      PIDController rearRightController,
//      Supplier<MecanumDriveWheelSpeeds> currentWheelSpeeds,
//      Consumer<MecanumDriveMotorVoltages> outputDriveVoltages,
//      Telemetry telemetry,
//      Subsystem... requirements) {
//    this(
//        trajectory,
//        pose,
//        feedforward,
//        kinematics,
//        xController,
//        yController,
//        thetaController,
//        false,
//        maxWheelVelocityMetersPerSecond,
//        frontLeftController,
//        rearLeftController,
//        frontRightController,
//        rearRightController,
//        currentWheelSpeeds,
//        outputDriveVoltages,
//            telemetry,
//        requirements);
//  }

  /**
   * Constructs a new MecanumControllerCommand that when executed will follow the provided
   * trajectory. The user should implement a velocity PID on the desired output wheel velocities.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary end-states.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the robot should be facing. This is sampled at each time
   *     step.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param outputWheelSpeeds A MecanumDriveWheelSpeeds object containing the output wheel speeds.
   * @param requirements The subsystems to require.
   */
  //@SuppressWarnings("this-escape")
  public MecanumControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      double maxWheelVelocityMetersPerSecond,
      Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "MecanumControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "MecanumControllerCommand");
    m_feedforward = new SimpleMotorFeedforward(0, 0, 0);
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "MecanumControllerCommand");

    m_controller =
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "MecanumControllerCommand"),
            requireNonNullParam(yController, "yController", "MecanumControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "MecanumControllerCommand"));

    m_desiredRotation =
        requireNonNullParam(desiredRotation, "desiredRotation", "MecanumControllerCommand");

    m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;

    m_frontLeftController = null;
    m_rearLeftController = null;
    m_frontRightController = null;
    m_rearRightController = null;

    m_currentWheelSpeeds = null;

    m_outputWheelSpeeds =
        requireNonNullParam(outputWheelSpeeds, "outputWheelSpeeds", "MecanumControllerCommand");

    m_outputDriveVoltages = null;

    m_usePID = false;

    addRequirements(requirements);
  }

  /**
   * Constructs a new MecanumControllerCommand that when executed will follow the provided
   * trajectory. The user should implement a velocity PID on the desired output wheel velocities.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary end-states.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain wheel.
   * @param outputWheelSpeeds A MecanumDriveWheelSpeeds object containing the output wheel speeds.
   * @param requirements The subsystems to require.
   */
  public MecanumControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      double maxWheelVelocityMetersPerSecond,
      Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        maxWheelVelocityMetersPerSecond,
        outputWheelSpeeds,
        requirements);
  }

  @Override
  public void initialize() {
    Trajectory.State initialState = m_trajectory.sample(0);

    double initialXVelocity =
        initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getCos();
    double initialYVelocity =
        initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getSin();

    MecanumDriveWheelSpeeds prevSpeeds =
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialXVelocity, initialYVelocity, 0.0));

    m_prevFrontLeftSpeedSetpoint = (prevSpeeds.frontLeftMetersPerSecond);
    m_prevRearLeftSpeedSetpoint = (prevSpeeds.rearLeftMetersPerSecond);
    m_prevFrontRightSpeedSetpoint = (prevSpeeds.frontRightMetersPerSecond);
    m_prevRearRightSpeedSetpoint = (prevSpeeds.rearRightMetersPerSecond);

    m_timer = new Timer();
    m_timer.reset();
  }

  @Override
  public void execute() {
    if(m_timer.get() < 0.0001)
      m_timer.start();
    double curTime = m_timer.get();

    DashServer.AddData("DriveTime", curTime);

    Trajectory.State desiredState = m_trajectory.sample(curTime);

    ChassisSpeeds targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState,
                desiredState.poseMeters.getRotation());//m_desiredRotation.get()); jd
    MecanumDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    double frontLeftCalculate = (targetWheelSpeeds.frontLeftMetersPerSecond);
    double backLeftCalculate = (targetWheelSpeeds.rearLeftMetersPerSecond);
    double frontRightCalculate = (targetWheelSpeeds.frontRightMetersPerSecond);
    double backRightCalculate = (targetWheelSpeeds.rearRightMetersPerSecond);

    targetWheelSpeeds.desaturate(m_maxWheelVelocityMetersPerSecond);

    m_frontLeftSpeedSetpoint = (targetWheelSpeeds.frontLeftMetersPerSecond);
    m_rearLeftSpeedSetpoint = (targetWheelSpeeds.rearLeftMetersPerSecond);
    m_frontRightSpeedSetpoint = (targetWheelSpeeds.frontRightMetersPerSecond);
    m_rearRightSpeedSetpoint = (targetWheelSpeeds.rearRightMetersPerSecond);

    Pose2d currentPose = m_pose.get();

    DashServer.AddData("currentPoseX",currentPose.getX());
    DashServer.AddData("currentPoseY",currentPose.getY());
    DashServer.AddData("currentPoseR",currentPose.getRotation().getDegrees());

    DashServer.AddData("desiredPoseX",desiredState.poseMeters.getX());
    DashServer.AddData("desiredPoseY",desiredState.poseMeters.getY());
    DashServer.AddData("desiredPoseR",desiredState.poseMeters.getRotation().getDegrees());

    DashServer.AddData("trajTimeSecond",desiredState.timeSeconds);
    DashServer.AddData("trajVelocityMPS",desiredState.velocityMetersPerSecond);
    DashServer.AddData("trajAccelerationMPSS",desiredState.accelerationMetersPerSecondSq);
    DashServer.AddData("trajCurvatureRdPM",desiredState.curvatureRadPerMeter);

    DashServer.AddData("frontLeftCalculate",frontLeftCalculate);
    DashServer.AddData("frontRightCalculate",frontRightCalculate);
    DashServer.AddData("backLeftCalculate",backLeftCalculate);
    DashServer.AddData("backRightCalculate",backRightCalculate);

    DashServer.AddData("maxWheelVelocityMPS",m_maxWheelVelocityMetersPerSecond);

    DashServer.AddData("frontLeftSpeedSetpoint",m_frontLeftSpeedSetpoint);
    DashServer.AddData("frontRightSpeedSetpoint",m_frontRightSpeedSetpoint);
    DashServer.AddData("rearLeftSpeedSetpoint",m_rearLeftSpeedSetpoint);
    DashServer.AddData("rearRightSpeedSetpoint",m_rearRightSpeedSetpoint);

    double frontLeftOutput;
    double rearLeftOutput;
    double frontRightOutput;
    double rearRightOutput;

    if (m_usePID) {
      final double frontLeftFeedforward =
          m_feedforward.calculate(m_prevFrontLeftSpeedSetpoint, m_frontLeftSpeedSetpoint);//.in(Volts);

      final double rearLeftFeedforward =
          m_feedforward.calculate(m_prevRearLeftSpeedSetpoint, m_rearLeftSpeedSetpoint);//.in(Volts);

      final double frontRightFeedforward =
          m_feedforward.calculate(m_prevFrontRightSpeedSetpoint, m_frontRightSpeedSetpoint);//.in(Volts);

      final double rearRightFeedforward =
          m_feedforward.calculate(m_prevRearRightSpeedSetpoint, m_rearRightSpeedSetpoint);//.in(Volts);

      frontLeftOutput =
          frontLeftFeedforward
              + m_frontLeftController.calculate(
                  m_currentWheelSpeeds.get().frontLeftMetersPerSecond,
                  m_frontLeftSpeedSetpoint/*.in(MetersPerSecond)*/);

      rearLeftOutput =
          rearLeftFeedforward
              + m_rearLeftController.calculate(
                  m_currentWheelSpeeds.get().rearLeftMetersPerSecond,
                  m_rearLeftSpeedSetpoint/*.in(MetersPerSecond)*/);

      frontRightOutput =
          frontRightFeedforward
              + m_frontRightController.calculate(
                  m_currentWheelSpeeds.get().frontRightMetersPerSecond,
                  m_frontRightSpeedSetpoint/*.in(MetersPerSecond)*/);

      rearRightOutput =
          rearRightFeedforward
              + m_rearRightController.calculate(
                  m_currentWheelSpeeds.get().rearRightMetersPerSecond,
                  m_rearRightSpeedSetpoint/*.in(MetersPerSecond)*/);

      m_outputDriveVoltages.accept(
          new MecanumDriveMotorVoltages(
              frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput));

    } else {
      m_outputWheelSpeeds.accept(
          new MecanumDriveWheelSpeeds(
              m_frontLeftSpeedSetpoint/*.in(MetersPerSecond)*/,
              m_frontRightSpeedSetpoint/*.in(MetersPerSecond)*/,
              m_rearLeftSpeedSetpoint/*.in(MetersPerSecond)*/,
              m_rearRightSpeedSetpoint/*.in(MetersPerSecond)*/));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
