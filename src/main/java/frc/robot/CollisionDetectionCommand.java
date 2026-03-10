// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollisionDetectionCommand extends Command {
  /** Creates a new CollisionDetection. */
  private final CommandSwerveDrivetrain m_swerve;
  private LinearAcceleration lastLinearAccelerationX;
  private LinearAcceleration lastLinearAccelerationY;

  private double xJerkLimit = 5;
  private double yJerkLimit = 5;
  private boolean firstPass = true;

  private final Alert collisionAlert = new Alert(
      "Collision Occurred",
      AlertType.kInfo);

  public CollisionDetectionCommand(CommandSwerveDrivetrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    collisionAlert.set(m_swerve.jerkLimitExceeded);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastLinearAccelerationX = MetersPerSecondPerSecond.of(0);
    lastLinearAccelerationY = MetersPerSecondPerSecond.of(0);
    firstPass = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LinearAcceleration linearAccerationX = m_swerve.getPigeon2().getAccelerationX().getValue();
    LinearAcceleration linearAccerationY = m_swerve.getPigeon2().getAccelerationY().getValue();

    double xJerk = linearAccerationX.minus(lastLinearAccelerationX).in(MetersPerSecondPerSecond);
    lastLinearAccelerationX = linearAccerationX;

    double yJerk = linearAccerationY.minus(lastLinearAccelerationY).in(MetersPerSecondPerSecond);
    lastLinearAccelerationY = linearAccerationY;

    if (!firstPass && (Math.abs(xJerk) > xJerkLimit || Math.abs(yJerk) > yJerkLimit))
      m_swerve.jerkLimitExceeded = true;

    Logger.log("COllDetect/xJerk", xJerk);
    Logger.log("COllDetect/yJerk", yJerk);
    Logger.log("COllDetect/xAccel", linearAccerationX.in(MetersPerSecondPerSecond));
    Logger.log("COllDetect/yAccel", linearAccerationY.in(MetersPerSecondPerSecond));
    Logger.log("COllDetect/detected", m_swerve.jerkLimitExceeded);
    
    firstPass = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
