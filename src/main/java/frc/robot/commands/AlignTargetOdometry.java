// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;

public class AlignTargetOdometry extends Command {
  /** Creates a new AlignToTagSet */
  CommandXboxController m_controller;
  private boolean feed;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final CommandSwerveDrivetrain m_drivetrain;
  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);
  public Pose2d targetPose = new Pose2d();
  private double rotationVal;
  private boolean aligning;
  private double angleToTarget;
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private SwerveRequest.FieldCentric drive;
  private final TripleShooterSubsystem shooter;

  public AlignTargetOdometry(
      CommandSwerveDrivetrain drivetrain,
      TripleShooterSubsystem shooter,
      SwerveRequest.FieldCentric drive,
      CommandXboxController controller,
      boolean feed) {

    m_drivetrain = drivetrain;
    m_controller = controller;
    this.feed = feed;
    this.drive = drive;
    this.shooter = shooter;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    // if (!lob) {
    targetPose = AllianceUtil.getHubPose();
    m_alignTargetPID.setTolerance(0.2);
    aligning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleToTarget = getAngleDegreesToTarget(targetPose, m_drivetrain.getState().Pose);

    shooter.setDistanceToHub(Constants.FieldConstants.blueHubPose.getTranslation()
        .getDistance(m_drivetrain.getState().Pose.getTranslation()));

    rotationVal = m_alignTargetPID.calculate(m_drivetrain.getState().Pose.getRotation().getDegrees(), angleToTarget);

    m_drivetrain.setControl(
        drive.withVelocityX(-m_controller.getLeftY() * MaxSpeed).withVelocityY(-m_controller.getLeftX() * MaxSpeed)
            // // negative X (left)
            .withRotationalRate(rotationVal * MaxAngularRate));

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

  public double getAngleDegreesToTarget(Pose2d targetPose, Pose2d robotPose) {
    double XDiff = targetPose.getX() - robotPose.getX();
    double YDiff = targetPose.getY() - robotPose.getY();
    return Units.radiansToDegrees(Math.atan2(YDiff, XDiff));

  }
}
