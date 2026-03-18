// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.RuntimeMXBean;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.ShiftDetectionCommand;
import frc.robot.commands.AprilTags.LimelightTagsMT1Update;
import frc.robot.commands.AprilTags.LimelightTagsMT2Update;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Logger;
import frc.robot.utils.LoopEvents;

public class Robot extends TimedRobot {
        private Command m_autonomousCommand;
        private final EventLoop m_eventLoop = new EventLoop();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        private final NetworkTable hubPoseTable = inst.getTable("HubPoses");

        private final StructPublisher<Pose2d> hubPoseRed = hubPoseTable.getStructTopic("HubPoseRed", Pose2d.struct)
                        .publish();
        private final StructPublisher<Pose2d> hubPoseBlue = hubPoseTable.getStructTopic("HubPoseBlue", Pose2d.struct)
                        .publish();

        private final RobotContainer m_robotContainer;
        private LoopEvents loopEvents;
        private boolean autoHasRun;

        Timer loopTimer = new Timer();
        /* log and replay timestamp and joystick data */
        private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
                        .withTimestampReplay()
                        .withJoystickReplay();

        public Robot() {
                hubPoseRed.set(FieldConstants.redHubPose);
                hubPoseBlue.set(FieldConstants.blueHubPose);
                // inst.flush();
                m_robotContainer = new RobotContainer();

                Logger.setOptions(
                                new DogLogOptions()
                                                .withNtPublish(true)
                                                .withCaptureNt(true)
                                                .withCaptureDs(true)
                                                .withLogExtras(true)
                                                .withUseLogThread(false)
                                                .withLogEntryQueueCapacity(2000));
                Logger.setEnabled(true);

                loopEvents = new LoopEvents(m_robotContainer.drivetrain, m_robotContainer.m_shooter, m_eventLoop);
                loopEvents.init();
                autoHasRun = false;

                if (RobotBase.isReal()) {

                        setDefaultLLPipelines();
                        runFrontTagUpdateCommands(true, true, true, true);
                        runLeftTagUpdateCommands(true, false, false, false);
                        runRightTagUpdateCommands(true, false, false, false);
                }
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

                autoHasRun = false;

                m_robotContainer.setForAutoShootValues();

                m_autonomousCommand = m_robotContainer.getAutonomousCommand();
                SmartDashboard.putString("AutoName", m_autonomousCommand.getName());
                if (m_autonomousCommand != null) {
                        CommandScheduler.getInstance().schedule(m_autonomousCommand);
                }
                // CommandScheduler.getInstance().schedule(
                // new CollisionDetectionCommand(m_robotContainer.drivetrain));

        }

        @Override
        public void autonomousPeriodic() {
        }

        @Override
        public void autonomousExit() {
                autoHasRun = true;

                if (DriverStation.isFMSAttached()) {
                        LimelightHelpers.triggerRewindCapture(m_robotContainer.m_llv.leftName, 20);
                        LimelightHelpers.triggerRewindCapture(m_robotContainer.m_llv.rightName, 20);
                }
        }

        @Override
        public void teleopInit() {
                // test vision in sim with left cam
                if (RobotBase.isSimulation()) {
                        LimelightHelpers.setPipelineIndex(CameraConstants.leftCamera.camname,
                                        CameraConstants.apriltagPipeline);
                        runLeftTagUpdateCommands(true, true, true, true);

                }

                if (RobotBase.isSimulation() && AllianceUtil.isBlueAlliance())
                        m_robotContainer.drivetrain.resetPose(new Pose2d(1, 3.5, new Rotation2d()));
                if (RobotBase.isSimulation() && AllianceUtil.isRedAlliance())
                        m_robotContainer.drivetrain.resetPose(new Pose2d(15, 3.5, new Rotation2d(Math.PI)));

                loopTimer.start();

                SignalLogger.setPath("/U/logs");

                if (m_autonomousCommand != null) {
                        CommandScheduler.getInstance().cancel(m_autonomousCommand);
                }

                m_robotContainer.m_shooter.hubIsActive = !autoHasRun;

                // if (DriverStation.isFMSAttached()) {
                CommandScheduler.getInstance()
                                .schedule(new ShiftDetectionCommand(m_robotContainer.m_shooter,
                                                m_robotContainer.m_leds));
                // }
                // new CollisionDetectionCommand(m_robotContainer.drivetrain));

        }

        @Override
        public void teleopPeriodic() {
        }

        @Override
        public void teleopExit() {

                if (DriverStation.isFMSAttached()) {
                        LimelightHelpers.triggerRewindCapture(m_robotContainer.m_llv.leftName, 130);
                        LimelightHelpers.triggerRewindCapture(m_robotContainer.m_llv.rightName, 130);
                }
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

        public void setDefaultLLPipelines() {

                LimelightHelpers.setPipelineIndex(CameraConstants.frontCamera.camname,
                                CameraConstants.apriltagPipeline);
                LimelightHelpers.setPipelineIndex(CameraConstants.leftCamera.camname,
                                CameraConstants.apriltagPipeline);
                LimelightHelpers.setPipelineIndex(CameraConstants.rightCamera.camname,
                                CameraConstants.apriltagPipeline);
                // LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
                // CameraConstants.fuelDetectorPipeline);

        }

        private void runFrontTagUpdateCommands(boolean runMT1, boolean runMT2, boolean mt1Correct, boolean mt2Correct) {
                if (runMT1) {
                        CommandScheduler.getInstance().schedule(
                                        new LimelightTagsMT1Update(m_robotContainer.m_llv,
                                                        m_robotContainer.m_llv.frontCam,
                                                        m_robotContainer.drivetrain).ignoringDisable(true));
                }
                if (runMT2) {
                        CommandScheduler.getInstance().schedule(new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                        m_robotContainer.m_llv.frontCam,
                                        m_robotContainer.drivetrain).ignoringDisable(true));
                }

                m_robotContainer.m_llv.useMT1[m_robotContainer.m_llv.frontCam] = mt1Correct;
                m_robotContainer.m_llv.useMT2[m_robotContainer.m_llv.frontCam] = mt2Correct;

        }

        private void runLeftTagUpdateCommands(boolean runMT1, boolean runMT2, boolean mt1Correct, boolean mt2Correct) {
                if (runMT1) {
                        CommandScheduler.getInstance().schedule(
                                        new LimelightTagsMT1Update(m_robotContainer.m_llv,
                                                        m_robotContainer.m_llv.leftCam,
                                                        m_robotContainer.drivetrain).ignoringDisable(true));
                }

                if (runMT2) {
                        CommandScheduler.getInstance().schedule(new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                        m_robotContainer.m_llv.leftCam,
                                        m_robotContainer.drivetrain).ignoringDisable(true));
                }
                m_robotContainer.m_llv.useMT1[m_robotContainer.m_llv.leftCam] = mt1Correct;
                m_robotContainer.m_llv.useMT2[m_robotContainer.m_llv.leftCam] = mt2Correct;
        }

        private void runRightTagUpdateCommands(boolean runMT1, boolean runMT2, boolean mt1Correct, boolean mt2Correct) {
                if (runMT1) {
                        CommandScheduler.getInstance().schedule(
                                        new LimelightTagsMT1Update(m_robotContainer.m_llv,
                                                        m_robotContainer.m_llv.leftCam,
                                                        m_robotContainer.drivetrain).ignoringDisable(true));
                }

                if (runMT2) {
                        CommandScheduler.getInstance().schedule(new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                        m_robotContainer.m_llv.leftCam,
                                        m_robotContainer.drivetrain).ignoringDisable(true));
                }
                m_robotContainer.m_llv.useMT1[m_robotContainer.m_llv.leftCam] = mt1Correct;
                m_robotContainer.m_llv.useMT2[m_robotContainer.m_llv.leftCam] = mt2Correct;
        }
}
