// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import util.DashServer;


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
public class MecanumDynamicControllerCommand extends CommandBase {
  private Timer m_timer;
  private final boolean m_usePID;
  private Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SimpleMotorFeedforward m_feedforward;
  private final MecanumDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
//  private final Supplier<Rotation2d> m_desiredRotation;
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

  //private RobotDataServer dataServer;

  Supplier<Pose2d> start;
  Pose2d end;
  TrajectoryConfig trajectoryConfig;


  //@SuppressWarnings("this-escape")
  public MecanumDynamicControllerCommand(
      Supplier<Pose2d> start,
      Pose2d end,
      TrajectoryConfig trajectoryConfig,
      Supplier<Pose2d> pose,
      MecanumDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
//      Rotation2d desiredRotation,
      double maxWheelVelocityMetersPerSecond,
      Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
      Subsystem... requirements) {

    this.start = start;
    this.end = end;
    this.trajectoryConfig = trajectoryConfig;

    m_pose = requireNonNullParam(pose, "pose", "MecanumControllerCommand");
    m_feedforward = new SimpleMotorFeedforward(0, 0, 0);
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "MecanumControllerCommand");

    m_controller =
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "MecanumControllerCommand"),
            requireNonNullParam(yController, "yController", "MecanumControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "MecanumControllerCommand"));

//    m_desiredRotation = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
//        requireNonNullParam(desiredRotation, "desiredRotation", "MecanumControllerCommand");

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

    //this.dataServer = dataServer;

    addRequirements(requirements);
  }


  @Override
  public void initialize() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(start.get(), end), trajectoryConfig);

    m_trajectory = requireNonNullParam(trajectory, "trajectory", "MecanumControllerCommand");

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

//    m_timer = new Timer();
//    m_timer.reset();
    startTime = 0;
  }

  private long startTime = 0;
  @Override
  public void execute() {
    if(startTime < 1)
      startTime = System.nanoTime();

    double curTime = (double) (System.nanoTime()-startTime) / 1E9;
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

    DashServer.AddData("currentPoseX", m_pose.get().getX());
    DashServer.AddData("currentPoseY", m_pose.get().getY());
    DashServer.AddData("currentPoseR", m_pose.get().getRotation().getDegrees());

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

    DashServer.AddData("frontLeftSpeedSetpoint",targetWheelSpeeds.frontLeftMetersPerSecond);
    DashServer.AddData("frontRightSpeedSetpoint",targetWheelSpeeds.frontRightMetersPerSecond);
    DashServer.AddData("rearLeftSpeedSetpoint",targetWheelSpeeds.rearLeftMetersPerSecond);
    DashServer.AddData("rearRightSpeedSetpoint",targetWheelSpeeds.rearRightMetersPerSecond);

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

//  @Override
//  public void end(boolean interrupted) {
//    //m_timer.stop();
//  }

  @Override
  public boolean isFinished() {
    //return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    return (double) (System.nanoTime()-startTime) / 1E9 > m_trajectory.getTotalTimeSeconds();
  }
}
