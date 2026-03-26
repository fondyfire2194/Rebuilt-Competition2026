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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AprilTags.LimelightTagsMT1Update;
import frc.robot.commands.AprilTags.LimelightTagsMT2Update;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LoopEvents;

public class Robot extends TimedRobot {
        private Command m_autonomousCommand;
        private final EventLoop m_eventLoop = new EventLoop();

        private BooleanEvent frontCamHasHubTag;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // private final NetworkTable hubPoseTable = inst.getTable("HubPoses");

        // private final StructPublisher<Pose2d> hubPoseRed =
        // hubPoseTable.getStructTopic("HubPoseRed", Pose2d.struct)
        // .publish();
        // private final StructPublisher<Pose2d> hubPoseBlue =
        // hubPoseTable.getStructTopic("HubPoseBlue", Pose2d.struct)
        // .publish();

        private final RobotContainer m_robotContainer;
        private LoopEvents loopEvents;
        private boolean autoHasRun;

        Timer loopTimer = new Timer();
        /* log and replay timestamp and joystick data */
        private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
                        .withTimestampReplay()
                        .withJoystickReplay();

        public Robot() {
                m_robotContainer = new RobotContainer();

        }

        @Override
        public void robotInit() {
                // hubPoseRed.set(FieldConstants.redHubPose);
                // hubPoseBlue.set(FieldConstants.blueHubPose);
                // // inst.flush();

                DogLog.setOptions(
                                new DogLogOptions()
                                                .withNtPublish(true)
                                                .withCaptureNt(false)
                                                .withCaptureDs(true)
                                                .withLogExtras(false)
                                                .withUseLogThread(false)
                                                .withLogEntryQueueCapacity(2000));
                DogLog.setEnabled(true);

                loopEvents = new LoopEvents(m_robotContainer.drivetrain, m_robotContainer.m_shooter, m_eventLoop);
                loopEvents.init();

                frontCamHasHubTag = new BooleanEvent(m_eventLoop,
                                () -> m_robotContainer.m_llv.getFrontCamSeesHubTags()).debounce(1);

                // frontCamHasHubTag.ifHigh(() ->
                // m_robotContainer.m_llv.setViewFinderLLPipelines());

                // frontCamHasHubTag.negate().ifHigh(() ->
                // m_robotContainer.m_llv.setDefaultLLPipelines());

                autoHasRun = false;

                m_robotContainer.m_llv.setDefaultLLPipelines();
                /**
                 * mainly for test at FDL.
                 * when the fron camera sees a hub tag, it will set the robot pose from MT1,
                 * reset useM!i and set useMT2
                 * If the front camera sees a hub tag, then the Viewfinder pipeline is set on
                 * the left and right cameras
                 * This is set back when the front camera doesn't see a hub tag.
                 * This allows the left and right cameras to add their results to the robot pose
                 * vision measurement.
                 * 
                 */
                CommandScheduler.getInstance().schedule(
                                runFrontMT1UpdatesCommand(),
                                runMT2UpdatesCommand());
                m_robotContainer.m_llv.useMT1 = true;
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

                m_robotContainer.m_llv.useMT1 = false;
                m_robotContainer.m_llv.useMT2 = true;

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

                if (RobotBase.isSimulation() && AllianceUtil.isBlueAlliance())
                        m_robotContainer.drivetrain.resetPose(new Pose2d(1, 3.5, new Rotation2d()));
                if (RobotBase.isSimulation() && AllianceUtil.isRedAlliance())
                        m_robotContainer.drivetrain.resetPose(new Pose2d(15, 3.5, new Rotation2d(Math.PI)));

                loopTimer.start();

                SignalLogger.setPath("/U/logs");

                if (m_autonomousCommand != null) {
                        CommandScheduler.getInstance().cancel(m_autonomousCommand);
                }

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

        public Command runMT2UpdatesCommand() {

                // return Commands.parallel(

                return new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                m_robotContainer.m_llv.frontCam,
                                m_robotContainer.drivetrain).ignoringDisable(true)
                                .withName("FrontMT2");

                // new LimelightTagsMT2Update(m_robotContainer.m_llv,
                // m_robotContainer.m_llv.leftCam,
                // m_robotContainer.drivetrain).ignoringDisable(true)
                // .withName("LeftMT2"),
                // new LimelightTagsMT2Update(m_robotContainer.m_llv,
                // m_robotContainer.m_llv.rightCam,
                // m_robotContainer.drivetrain).ignoringDisable(true)
                // .withName("RightMT2"));
        }

        public Command runFrontMT2UpdatesCommand() {
                return new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                m_robotContainer.m_llv.frontCam,
                                m_robotContainer.drivetrain).ignoringDisable(true)
                                .withName("FrontMT2");

        }

        public Command runFrontMT1UpdatesCommand() {
                return new LimelightTagsMT1Update(m_robotContainer.m_llv,
                                m_robotContainer.m_llv.frontCam,
                                m_robotContainer.drivetrain).ignoringDisable(true)
                                .withName("FrontMT1");
        }
}
