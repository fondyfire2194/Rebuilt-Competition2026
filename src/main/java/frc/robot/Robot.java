// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightTagsMT2Update;
import frc.robot.utils.LoopEvents;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final EventLoop m_eventLoop = new EventLoop();
    // StructPublisher<Pose2d> rHposePublisher = NetworkTableInstance.getDefault()
    // .getStructTopic("BlueHubPose", Pose2d.struct).publish();
    // StructPublisher<Pose2d> bHposePublisher = NetworkTableInstance.getDefault()
    // .getStructTopic("RedHubPose", Pose2d.struct).publish();

    private final RobotContainer m_robotContainer;
    private LoopEvents loopEvents;
    private boolean autoHasRun;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        loopEvents = new LoopEvents(m_robotContainer.drivetrain, m_robotContainer.m_shooter, m_eventLoop);
        loopEvents.init();
        autoHasRun = false;
    }

    @Override
    public void robotPeriodic() {

        m_eventLoop.poll();

        m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run();

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        SmartDashboard.putString("AutoName", m_autonomousCommand.getName());
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        CommandScheduler.getInstance().schedule(
                new LimelightTagsMT2Update(m_robotContainer.m_llv, m_robotContainer.m_llv.frontCam,
                        m_robotContainer.drivetrain),
                new LimelightTagsMT2Update(m_robotContainer.m_llv, m_robotContainer.m_llv.frontCam,
                        m_robotContainer.drivetrain),
                new LimelightTagsMT2Update(m_robotContainer.m_llv, m_robotContainer.m_llv.frontCam,
                        m_robotContainer.drivetrain));

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        autoHasRun = true;
    }

    @Override
    public void teleopInit() {
        LimelightHelpers.setPipelineIndex(CameraConstants.frontCamera.camname, CameraConstants.apriltagPipeline);
        LimelightHelpers.setPipelineIndex(CameraConstants.leftCamera.camname, CameraConstants.apriltagPipeline);
        LimelightHelpers.setPipelineIndex(CameraConstants.rightCamera.camname, CameraConstants.apriltagPipeline);

        SignalLogger.setPath("media/sda1/logs");
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        if (!autoHasRun) {
            CommandScheduler.getInstance().schedule(
                    new LimelightTagsMT2Update(m_robotContainer.m_llv, m_robotContainer.m_llv.frontCam,
                            m_robotContainer.drivetrain),
                    new LimelightTagsMT2Update(m_robotContainer.m_llv, m_robotContainer.m_llv.leftCam,
                            m_robotContainer.drivetrain),
                    new LimelightTagsMT2Update(m_robotContainer.m_llv, m_robotContainer.m_llv.rightCam,
                            m_robotContainer.drivetrain));
            m_robotContainer.m_llv.useMT2 = true;
        }

        if (RobotBase.isSimulation())
            m_robotContainer.drivetrain.resetPose(new Pose2d(13.5, 3.1, new Rotation2d(Math.PI)));
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
