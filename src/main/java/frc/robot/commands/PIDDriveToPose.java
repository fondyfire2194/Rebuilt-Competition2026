// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PIDDriveToPose extends Command {
  private final CommandSwerveDrivetrain swerve;
  private Pose2d target;

  private final PIDController xController = new PIDController(2.8, 0, 0);
  private final PIDController yController = new PIDController(2.8, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                      // top // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                    // second

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

  /** Creates a new PIDDriveToPose. */
  public PIDDriveToPose(CommandSwerveDrivetrain swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;

    xController.setTolerance(0.02); /// 0.0125
    yController.setTolerance(0.02);
    xController.setIZone(0.05);
    xController.setIntegratorRange(-0.07, 0.07);
    yController.setIZone(0.05);
    yController.setIntegratorRange(-0.07, 0.07);

    xController.setI(0.03);
    yController.setI(0.03);
    xController.setP(3);
    yController.setP(3);

    thetaController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  public PIDDriveToPose(CommandSwerveDrivetrain swerve, Pose2d target, double toleranceTranslation,
      double toleranceAngle) {
    this.swerve = swerve;
    this.target = target;

    xController.setTolerance(toleranceTranslation);
    yController.setTolerance(toleranceTranslation);
    xController.setI(0);
    yController.setI(0);
    xController.setP(3.8); // 0.3.4
    yController.setP(3.8);

    // xController.setIZone(0.2);
    // xController.setIntegratorRange(-0.05, 0.05);
    // yController.setIZone(0.2);
    // yController.setIntegratorRange(-0.05, 0.05);

    thetaController.setTolerance(Rotation2d.fromDegrees(toleranceAngle).getRadians());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    thetaController.setSetpoint(target.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = xController.calculate(swerve.getState().Pose.getX());
SmartDashboard.putNumber("VVVVEEELLLXXXX", xVel);
    double yVel = yController.calculate(swerve.getState().Pose.getY());

SmartDashboard.putNumber("VVVVEEELLLYYYYY", yVel);
    swerve.setControl( drive
        .withVelocityX(-xVel)
        .withVelocityY(-yVel)
        .withRotationalRate(
            thetaController.calculate(swerve.getState().Pose.getRotation().getRadians())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
