// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;

public class AutoAlignHub extends Command {

  private final CommandSwerveDrivetrain m_swerve;
  private final TripleShooterSubsystem m_shooter;
  private final double m_toleranceDegrees;
  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);

  private SwerveRequest.FieldCentric drive;
  public Pose2d targetPose = new Pose2d();
  private double rotationVal;
  private boolean aligning;
  private Timer elapsedTime;

  private double angleToTarget;

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private boolean alignedToTarget;

  public AutoAlignHub(
      CommandSwerveDrivetrain swerve, TripleShooterSubsystem shooter, double toleranceDegrees) {

    m_swerve = swerve;
    m_shooter = shooter;
    m_toleranceDegrees = toleranceDegrees;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    m_alignTargetPID.setTolerance(m_toleranceDegrees);
    m_alignTargetPID.setIZone(1);
    m_alignTargetPID.setIntegratorRange(-.01, .01);
    m_alignTargetPID.setI(.0001);
    m_alignTargetPID.reset();
    targetPose = AllianceUtil.getHubPose();
    m_alignTargetPID.setTolerance(0.2);

    aligning = true;
    elapsedTime = new Timer();
    elapsedTime.reset();
    elapsedTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleToTarget = getAngleDegreesToTarget(targetPose, m_swerve.getState().Pose);

    m_shooter.setDistanceToHub(targetPose.getTranslation()
        .getDistance(m_swerve.getState().Pose.getTranslation()));

    rotationVal = m_alignTargetPID.calculate(m_swerve.getState().Pose.getRotation().getDegrees(), angleToTarget);

    m_swerve.setControl(drive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(rotationVal * MaxAngularRate));

    alignedToTarget = Math.abs(angleToTarget) < m_toleranceDegrees;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_alignTargetPID.reset();
   m_swerve.setControl(drive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alignedToTarget || elapsedTime.hasElapsed(2) ||
                          RobotBase.isSimulation();
  }

  public double getAngleDegreesToTarget(Pose2d targetPose, Pose2d robotPose) {
    double XDiff = targetPose.getX() - robotPose.getX();
    double YDiff = targetPose.getY() - robotPose.getY();
    return Units.radiansToDegrees(Math.atan2(YDiff, XDiff));
  }
}
