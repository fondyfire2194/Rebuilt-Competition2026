// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederSetpoints;
import frc.robot.Robot;
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
  private boolean m_bypassInterlocks;
  private Timer beltTimer = new Timer();
  private boolean okToShoot;
  private boolean lookForPulse;

  public ShootCommand(TripleShooterSubsystem shooter, HoodSubsystem hood, FeederSubsystem feeder,
      CommandSwerveDrivetrain swerve, boolean bypassInterlocks) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_hood = hood;
    m_feeder = feeder;
    m_swerve = swerve;
    m_bypassInterlocks = bypassInterlocks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    okToShoot = false;
    beltTimer.start();
    lookForPulse = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.bypassShootInterlocks = m_bypassInterlocks;

    if (m_shooter.bypassShootInterlocks
        || (m_shooter.allVelocityInTolerance()
            && m_hood.isPositionWithinTolerance() && (m_swerve.alignedToTarget))) {
      okToShoot = true;
    }

    if (okToShoot) {

      m_feeder.runFeederRollerAtVelocity();

      if (RobotBase.isSimulation() ||
          Math.abs(m_feeder.feederRollerMotor.getEncoder().getVelocity()) > FeederSetpoints.rollerSpeedToStartBelt)

      {

        if (!lookForPulse && beltTimer.get() > m_feeder.beltInitialShootTime) {
          lookForPulse = true;
          beltTimer.reset();
        }

        if (lookForPulse && beltTimer.get() > m_feeder.beltStartPulseTime)
          m_feeder.pulse = true;

        if (lookForPulse && beltTimer.get() > m_feeder.beltStopPulseTime) {
          m_feeder.pulse = false;
          beltTimer.reset();
        }

        m_feeder.pulse = false;// force no belt reverse pulse

        if (!m_feeder.pulse)
          m_feeder.runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint);
        else
          m_feeder.pulseBelt();
      }
    }

    if (RobotBase.isSimulation())
      Robot.fuelRobotSim.launchFuel();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stopFeederBeltMotor();
    m_feeder.stopFeederRollerMotor();
    okToShoot = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
