// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederSetpoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final TripleShooterSubsystem m_shooter;
  private final HoodSubsystem m_hood;
  private final FeederSubsystem m_feeder;
  private final CommandSwerveDrivetrain m_swerve;

  public ShootCommand(TripleShooterSubsystem shooter, HoodSubsystem hood, FeederSubsystem feeder,
      CommandSwerveDrivetrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_hood = hood;
    m_feeder = feeder;
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.runAllVelocityVoltage();

    if (m_shooter.bypassShootInterlocks
        || m_shooter.allVelocityInTolerance() && m_hood.isPositionWithinTolerance() && m_swerve.alignedToTarget) {
      m_feeder.runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint);
      m_feeder.runFeederRollerMotor(FeederSetpoints.kFeedRollerSetpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stopFeederBeltMotor();
    m_feeder.stopFeederRollerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
