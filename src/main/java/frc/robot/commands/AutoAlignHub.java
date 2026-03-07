// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.Logger;

public class AutoAlignHub extends Command {

  private final CommandSwerveDrivetrain m_swerve;
  private final TripleShooterSubsystem m_shooter;
  private final double m_toleranceDegrees;
  private SwerveRequest.FieldCentric drive;
  public Pose2d targetPose = new Pose2d();
  private double rotationVal;
  private Timer elapsedTime;
  private double distanceToHub;
  private double angleToTarget;

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private boolean alignedToTarget;
  private double tempI;

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
    targetPose = AllianceUtil.getHubPose();

    m_swerve.isAligning = true;
    elapsedTime = new Timer();
    elapsedTime.reset();
    elapsedTime.start();
    tempI = m_swerve.m_alignTargetPID.getI();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleToTarget = getAngleDegreesToTarget(targetPose, m_swerve.getState().Pose);
    distanceToHub = targetPose.getTranslation()
        .getDistance(m_swerve.getState().Pose.getTranslation());
    m_shooter.setDistanceToHub(distanceToHub);

    if (Math.abs(m_swerve.m_alignTargetPID.getError()) > m_swerve.alignIzone) {
      m_swerve.m_alignTargetPID.setI(0);
    } else
      m_swerve.m_alignTargetPID.setI(tempI);

    rotationVal = m_swerve.m_alignTargetPID.calculate(m_swerve.getState().Pose.getRotation().getDegrees(),
        angleToTarget);

    m_swerve.setControl(drive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(rotationVal * MaxAngularRate));

    alignedToTarget = Math.abs(angleToTarget) < m_toleranceDegrees;

    Logger.log("AlignedToHub", m_swerve.alignedToTarget);
    Logger.log("AlignError", m_swerve.m_alignTargetPID.getError());
    Logger.log("AlignDistance", distanceToHub);
    Logger.log("AlignAngle", angleToTarget);
    Logger.log("AlignHubAngle", HoodSubsystem.autoTargetAngle);
    Logger.log("AlighShootSpeed", m_shooter.autoSetTargetRPM);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.m_alignTargetPID.reset();
    m_swerve.isAligning = false;
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
