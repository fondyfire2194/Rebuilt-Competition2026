// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignHub extends Command {

  private final CommandSwerveDrivetrain m_swerve;
  private final boolean m_endAtTargets;
  private final double m_toleranceDegrees;
  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);

  private double rotationVal;
  private Timer elapsedTime;

  public AutoAlignHub(
      CommandSwerveDrivetrain swerve, double toleranceDegrees, boolean endAtTargets) {

    m_swerve = swerve;
    m_endAtTargets = endAtTargets;
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
   // m_swerve.targetPose = AllianceUtil.getHubPose();
    elapsedTime = new Timer();
    elapsedTime.reset();
    elapsedTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   // rotationVal = m_alignTargetPID.calculate(m_swerve.getAngleDegrees(), m_swerve.getAngleDegreesToTarget());

   // m_swerve.alignedToTarget = m_alignTargetPID.atSetpoint();

    // m_swerve.drive(
    //     0, 0,
    //     rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
    //     true,
    //     true,
    //     false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_alignTargetPID.reset();
    //m_swerve.drive(0, 0, 0, false, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endAtTargets;// && (m_swerve.alignedToTarget || elapsedTime.hasElapsed(2) || RobotBase.isSimulation());
  }
}
