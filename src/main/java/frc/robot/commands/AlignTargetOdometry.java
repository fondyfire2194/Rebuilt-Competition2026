// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShootingData;
import frc.robot.utils.geometry.AllianceFlipUtil;

public class AlignTargetOdometry extends Command {
  /** Creates a new AlignToTagSet */
  CommandXboxController m_controller;
  private boolean feed;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final CommandSwerveDrivetrain m_drivetrain;
  private final HoodSubsystem hood;

  private DoubleSubscriber kp;
  private DoubleSubscriber ki;
  private DoubleSubscriber kd;

  public PIDController m_alignTargetPID;
  public Pose2d targetPose = new Pose2d();
  private double rotationVal;
  private boolean aligning;
  private double angleToTarget;

  private SwerveRequest.FieldCentric drive;
  private final TripleShooterSubsystem shooter;
  private double distanceToHub;
  private boolean passing;

  public AlignTargetOdometry(
      CommandSwerveDrivetrain drivetrain,
      TripleShooterSubsystem shooter,
      HoodSubsystem hood,
      SwerveRequest.FieldCentric drive,
      CommandXboxController controller,
      boolean feed) {

    m_drivetrain = drivetrain;
    m_controller = controller;
    this.hood = hood;
    this.feed = feed;
    this.drive = drive;
    this.shooter = shooter;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kp = DogLog.tunable("Align/PGain", .03, newKp -> m_alignTargetPID.setP(newKp));
    ki = DogLog.tunable("Align/IGain", .0, newKi -> m_alignTargetPID.setI(newKi));
    kd = DogLog.tunable("Align/DGain", .0, newKd -> m_alignTargetPID.setI(newKd));
    m_alignTargetPID = new PIDController(kp.get(), ki.get(), kd.get());

    m_alignTargetPID.enableContinuousInput(-180, 180);
    if (!passing) {
      targetPose = AllianceUtil.getHubPose();
    }
    m_alignTargetPID.setTolerance(0.2);
    aligning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    angleToTarget = getAngleDegreesToTarget(targetPose, m_drivetrain.getState().Pose);

    passing = AllianceFlipUtil
        .applyX(m_drivetrain.getState().Pose.getX()) > FieldConstants.LinesVertical.hubCenter;

    distanceToHub = targetPose.getTranslation()
        .getDistance(m_drivetrain.getState().Pose.getTranslation());

    shooter.autoSetTargetRPM = passing ? ShootingData.passingShooterSpeedMap.get(distanceToHub)
        : ShootingData.shooterSpeedMap.get(distanceToHub);

    HoodSubsystem.autoTargetAngle = passing ? ShootingData.passingHoodAngleMap.get(distanceToHub).getDegrees()
        : ShootingData.hoodAngleMap.get(distanceToHub).getDegrees();

    rotationVal = m_alignTargetPID.calculate(m_drivetrain.getState().Pose.getRotation().getDegrees(), angleToTarget);

    m_drivetrain.setControl(
        drive.withVelocityX(-m_controller.getLeftY() * RobotConstants.MaxSpeed)
            .withVelocityY(-m_controller.getLeftX() * RobotConstants.MaxSpeed)
            // // negative X (left)
            .withRotationalRate(rotationVal * RobotConstants.MaxAngularRate));

    m_drivetrain.alignedToTarget = Math.abs(angleToTarget) < m_drivetrain.shootTolerance;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aligning = false;

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
