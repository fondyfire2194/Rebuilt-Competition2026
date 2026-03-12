// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.CollisionDetectionCommand;
import frc.robot.commands.ShiftDetectionCommand;
import frc.robot.commands.AprilTags.LimelightTagsMT2Update;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FuelSim;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Logger;
import frc.robot.utils.LoopEvents;
import frc.robot.utils.SimRobotFuelSim;

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

        public static FuelSim fuelSim;

        public static SimRobotFuelSim fuelRobotSim;

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
                fuelSim = new FuelSim("FuelSim");
                configureFuelSim();
                fuelRobotSim = new SimRobotFuelSim(
                                fuelSim, m_robotContainer.drivetrain, m_robotContainer.m_hood,
                                m_robotContainer.m_shooter);
                configureFuelSimRobot(() -> m_robotContainer.m_intake.intakeRunning(), () -> fuelRobotSim.intakeFuel());

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
                                                m_robotContainer.drivetrain),
                                new ShiftDetectionCommand(m_robotContainer.m_shooter, m_robotContainer.m_leds),
                                new CollisionDetectionCommand(m_robotContainer.drivetrain));

                if (RobotBase.isSimulation())
                        configureFuelSim();
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

                // Logger.log("DEPOTPASSINGPOSE", AllianceUtil.getDepotPassingPose());
                // Logger.log("OUTPOSTPASSINGPOSE", AllianceUtil.getOutpostPassingPose());
                // Logger.log("ALLIANCEBLUE", AllianceUtil.isBlueAlliance());
                // Logger.log("ALLIANCERED", AllianceUtil.isRedAlliance());

                if (RobotBase.isSimulation() && AllianceUtil.isBlueAlliance())
                        m_robotContainer.drivetrain.resetPose(new Pose2d(1, 3.5, new Rotation2d()));
                if (RobotBase.isSimulation() && AllianceUtil.isRedAlliance())
                        m_robotContainer.drivetrain.resetPose(new Pose2d(15, 3.5, new Rotation2d(Math.PI)));
          // m_robotContainer.drivetrain.resetPose(new Pose2d(3.5, 2, Rotation2d.fromDegrees(45)));
           m_robotContainer.drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            
                loopTimer.start();
                

                if (RobotBase.isReal()) {
                        LimelightHelpers.setPipelineIndex(CameraConstants.frontCamera.camname,
                                        CameraConstants.apriltagPipeline);
                        LimelightHelpers.setPipelineIndex(CameraConstants.leftCamera.camname,
                                        CameraConstants.apriltagPipeline);
                        LimelightHelpers.setPipelineIndex(CameraConstants.rightCamera.camname,
                                        CameraConstants.apriltagPipeline);
                        LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
                                        CameraConstants.fuelDetectorPipeline);

                }
                // SignalLogger.setPath("media/sda1/logs");
                SignalLogger.setPath("/U/SigLogs/logs");
                if (m_autonomousCommand != null) {
                        CommandScheduler.getInstance().cancel(m_autonomousCommand);
                }

                if (RobotBase.isReal() && !autoHasRun) {
                        CommandScheduler.getInstance().schedule(
                                        new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                                        m_robotContainer.m_llv.frontCam,
                                                        m_robotContainer.drivetrain),
                                        new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                                        m_robotContainer.m_llv.leftCam,
                                                        m_robotContainer.drivetrain),
                                        new LimelightTagsMT2Update(m_robotContainer.m_llv,
                                                        m_robotContainer.m_llv.rightCam,
                                                        m_robotContainer.drivetrain));

                        m_robotContainer.m_llv.useMT2 = true;
                }

                 m_robotContainer.m_shooter.hubIsActive = !autoHasRun;
                // CommandScheduler.getInstance()
                //                 .schedule(new ShiftDetectionCommand(m_robotContainer.m_shooter,
                //                                 m_robotContainer.m_leds),
                //                                 new CollisionDetectionCommand(m_robotContainer.drivetrain));

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
                fuelSim.updateSim();
        }

        public double getAngleDegreesToTarget(Pose2d targetPose, Pose2d robotPose) {
                double XDiff = targetPose.getX() - robotPose.getX();
                double YDiff = targetPose.getY() - robotPose.getY();
                return Units.radiansToDegrees(Math.atan2(YDiff, XDiff));
        }

        void configureFuelSim() {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
                fuelSim.start();
                fuelSim.enableAirResistance();
        }

        public void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
                fuelSim.registerRobot(
                                Dimensions.FULL_WIDTH.in(Meters),
                                Dimensions.FULL_LENGTH.in(Meters),
                                Dimensions.BUMPER_HEIGHT.in(Meters),
                                () -> m_robotContainer.drivetrain.getState().Pose,
                                () -> m_robotContainer.drivetrain.getState().Speeds);
                fuelSim.registerIntake(
                                -Dimensions.FULL_LENGTH.div(2).in(Meters),
                                Dimensions.FULL_LENGTH.div(2).in(Meters),
                                -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
                                -Dimensions.FULL_WIDTH.div(2).in(Meters),
                                () -> m_robotContainer.m_intake.intakeRunning(),
                                intakeCallback);

        }

}
