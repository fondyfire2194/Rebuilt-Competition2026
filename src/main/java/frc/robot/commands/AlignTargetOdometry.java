// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final CommandSwerveDrivetrain m_swerve;
  private final HoodSubsystem hood;

  public Pose2d targetPose = new Pose2d();
  private double rotationVal;
  private double targetDegrees;

  private SwerveRequest.FieldCentric drive;
  private final TripleShooterSubsystem shooter;
  private double distanceToTarget;
  private boolean passing;

  private final double m_toleranceDegrees;
  private double lastTargetDegrees;
  private int delayCount;
  private int delayLimit = 10;// 10*20ms
  private final double deadband = .02;

  public AlignTargetOdometry(
      CommandSwerveDrivetrain swerve,
      TripleShooterSubsystem shooter,
      HoodSubsystem hood,
      SwerveRequest.FieldCentric drive,
      CommandXboxController controller,
      double toleranceDegrees) {

    m_swerve = swerve;
    m_controller = controller;
    this.hood = hood;
    this.drive = drive;
    this.shooter = shooter;
    m_toleranceDegrees = toleranceDegrees;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.isAligning = true;
    m_swerve.m_alignTargetPID.reset();
    delayCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // allow time for conditions to settle
    // rotation is not applied before this
    if (delayCount <= delayLimit)
      delayCount++;

    passing = AllianceFlipUtil
        .applyX(m_swerve.getState().Pose.getX()) > FieldConstants.LinesVertical.hubCenter;
    if (!passing) {
      targetPose = AllianceUtil.getHubPose();
    } else
      targetPose = AllianceUtil.getPassingTargetPose(m_swerve.getState().Pose);

    distanceToTarget = targetPose.getTranslation()
        .getDistance(m_swerve.getState().Pose.getTranslation());

    shooter.setAutoSetTargetRPM(passing ? ShootingData.passingShooterSpeedMap.get(distanceToTarget)
        : ShootingData.shooterSpeedMap.get(distanceToTarget));

    hood.setAutoTargetAngle(passing ? ShootingData.passingHoodAngleMap.get(distanceToTarget).getDegrees()
        : ShootingData.hoodAngleMap.get(distanceToTarget).getDegrees());

    targetDegrees = getAngleDegreesToTarget(targetPose, m_swerve.getState().Pose);

    if (targetDegrees != lastTargetDegrees) {
      m_swerve.m_alignTargetPID.setSetpoint(targetDegrees);
      lastTargetDegrees = targetDegrees;
    }

    if (m_swerve.alignedToTarget || Math.abs(m_swerve.m_alignTargetPID.getError()) > m_swerve.alignIzone) {
      m_swerve.m_alignTargetPID.setIntegratorRange(0, 0);
    } else
      m_swerve.m_alignTargetPID.setIntegratorRange(-.1, .1);

    rotationVal = m_swerve.m_alignTargetPID.calculate(m_swerve.getState().Pose.getRotation().getDegrees(),
        targetDegrees);

    if (delayCount < delayLimit)
      rotationVal = 0;

    double translation = m_controller.getLeftY();
    double strafe = m_controller.getLeftX();
    if (Math.abs(translation) < deadband)
      translation = 0;
    if (Math.abs(strafe) < deadband)
      strafe = 0;

    m_swerve.setControl(
        drive.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withVelocityX(-translation * RobotConstants.MaxSpeed)
            .withVelocityY(-strafe * RobotConstants.MaxSpeed)
            // // negative X (left)
            .withRotationalRate(rotationVal * RobotConstants.MaxAngularRate));

    m_swerve.alignedToTarget = Math.abs(m_swerve.m_alignTargetPID.getError()) < m_toleranceDegrees;

    DogLog.log("Align/AlignedToHub", m_swerve.alignedToTarget);
    DogLog.log("Align/AlignError", m_swerve.m_alignTargetPID.getError());
    DogLog.log("Align/AlignDistance", distanceToTarget);
    DogLog.log("Align/AlignAngle", targetDegrees);
    DogLog.log("Align/HoodAngle", HoodSubsystem.autoTargetAngle);
    DogLog.log("Align/ShootSpeed", shooter.autoSetTargetRPM);
    DogLog.log("Align/Passing", passing);
    DogLog.log("Align/TargetPose", targetPose);
    DogLog.log("Align/AccumIntegral", m_swerve.m_alignTargetPID.getAccumulatedError());
    DogLog.log("Align/RotationVal", rotationVal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.isAligning = false;
    m_swerve.m_alignTargetPID.reset();
    delayCount = 0;
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
