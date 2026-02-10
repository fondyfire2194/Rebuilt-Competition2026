// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SD;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    StructPublisher<Pose2d> rHposePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("BlueHubPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> bHposePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RedHubPose", Pose2d.struct).publish();

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    private double angleToTarget;

    private double distanceToTarget;

    public Robot() {
        m_robotContainer = new RobotContainer();
        bHposePublisher.set(Constants.FieldConstants.blueHubPose);
        rHposePublisher.set(Constants.FieldConstants.redHubPose);
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run();

        if (AllianceUtil.isBlueAlliance()) {
            angleToTarget = getAngleDegreesToTarget(Constants.FieldConstants.blueHubPose,
                    m_robotContainer.drivetrain.getState().Pose);
            distanceToTarget = Constants.FieldConstants.blueHubPose.getTranslation()
                    .getDistance(m_robotContainer.drivetrain.getState().Pose.getTranslation());
        }

        if (AllianceUtil.isRedAlliance()) {
            angleToTarget = getAngleDegreesToTarget(Constants.FieldConstants.redHubPose,
                    m_robotContainer.drivetrain.getState().Pose);
            distanceToTarget = Constants.FieldConstants.redHubPose.getTranslation()
                    .getDistance(m_robotContainer.drivetrain.getState().Pose.getTranslation());
        }

        SD.sd2("AngleToTgt", angleToTarget
                - m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        SD.sd2("DistToTgt", Units.metersToInches(distanceToTarget));

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

        if (CameraConstants.frontCamera.isUsed)
            LimelightHelpers.SetRobotOrientation(CameraConstants.frontCamera.camname,
                    m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees(),
                    m_robotContainer.drivetrain.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0,
                    0);

        if (CameraConstants.leftCamera.isUsed)
            LimelightHelpers.SetRobotOrientation(CameraConstants.leftCamera.camname,
                    m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees(),
                    m_robotContainer.drivetrain.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0,
                    0);
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        SmartDashboard.putString("AutName", m_autonomousCommand.getName());
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {

        SignalLogger.setPath("media/sda1/logs");
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    

    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public double getAngleDegreesToTarget(Pose2d targetPose, Pose2d robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Units.radiansToDegrees(Math.atan2(YDiff, XDiff));

    }
}
