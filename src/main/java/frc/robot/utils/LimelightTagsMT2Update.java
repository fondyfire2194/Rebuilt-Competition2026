// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers.IMUData;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class LimelightTagsMT2Update extends Command {

    private final CommandSwerveDrivetrain m_swerve;
    private final CameraConstants.CameraValues m_cam;
    private boolean m_useMegaTag2;
    boolean rejectMT2Update;
    boolean rejectMT1Update;
    CameraData m_data;
    private final double AMBIGUITY_CUTOFF = 0.7;
    private final double DISTANCE_CUTOFF = 4.0;
    private final double DISTANCE_STDDEVS_SCALAR = 2;
    private final double ROTATION_RATE_CUTOFF = 720;

    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();

    StructPublisher<Pose2d> mt2PosePublisher;

    private boolean showData = true;

    public LimelightTagsMT2Update(CameraConstants.CameraValues cam, CameraData data, CommandSwerveDrivetrain swerve) {
        m_cam = cam;
        m_swerve = swerve;
        m_data = data;
        m_data.inhibitVision = false;
        setCamToRobotOffset(m_cam);
        if (m_cam.camname == "limelight-front") {
            CameraConstants.arrayPublisher.accept(CameraConstants.camPoses);
        }

        mt2PosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(m_cam.camname + " MT2Pose", Pose2d.struct).publish();

    }

    public void setCamToRobotOffset(Constants.CameraConstants.CameraValues cam) {
        LimelightHelpers.setCameraPose_RobotSpace(
                cam.camname,
                cam.camPose.getX(),
                cam.camPose.getY(),
                cam.camPose.getZ(),
                cam.camPose.getRotation().getY(),
                cam.camPose.getRotation().getX(),
                cam.camPose.getRotation().getZ());
    }

    public void setUseMegatag2(boolean on) {
        m_useMegaTag2 = on;
    }

    public int getIMUMode(String camName) {
        return (int) LimelightHelpers.getLimelightNTDouble(camName, "imu_set");
    }

    public void setLLRobotOrientation() {
        LimelightHelpers.SetRobotOrientation(m_cam.camname,
                m_swerve.getRotation3d().toRotation2d().getDegrees(),
                m_swerve.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
    }

    public void setLLRobotOrientationLL4IMU() {
        IMUData imuData = LimelightHelpers.getIMUData(m_cam.camname);
        LimelightHelpers.SetRobotOrientation(m_cam.camname, imuData.Yaw, 0, 0, 0, 0, 0);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        m_data.limeLightExists = LimelightHelpers.getLimelightNTTable(m_cam.camname).containsKey("tv");
        m_data.isActive = m_data.limeLightExists;

        setLLRobotOrientation();
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cam.camname);
        m_data.mt2Pose = mt2.pose;
        if (mt2.rawFiducials.length > 0) {
            m_data.MT2ambiguity = mt2.rawFiducials[0].ambiguity;
            m_data.MT2distToCamera = m_swerve.distanceLimelightToEstimator;
            m_data.numberMT2Pose = mt2.tagCount;

            if (showData) {
                mt2PosePublisher.set(mt2.pose);// send to network tables
                SmartDashboard.putNumber(m_cam.camname + " MT2 Tag Count", mt2.tagCount);
                SmartDashboard.putNumber(m_cam.camname + " MT2 Abiguity", m_data.MT2ambiguity);
                SmartDashboard.putNumber(m_cam.camname + " MT2 Dist To Cam", m_data.MT2distToCamera);
                SmartDashboard.putNumber(m_cam.camname + " MT2 Rotation To Cam",
                        m_data.mt2Pose.getRotation().getDegrees());

            }
        }

        setUseMegatag2(m_data.m_useMegaTag2);

        if (m_useMegaTag2) {
            if (mt2.rawFiducials.length > 0)
                m_swerve.distanceLimelightToEstimator = mt2.rawFiducials[0].distToCamera;

            rejectMT2Update = m_data.inhibitVision || mt2.tagCount == 0 || inFieldCheck(m_data.mt2Pose)
                    || Math.abs(m_swerve.getPigeon2().getAngularVelocityXDevice()
                            .getValueAsDouble()) > ROTATION_RATE_CUTOFF
                    || (mt2.tagCount == 1 && mt2.rawFiducials[0].ambiguity > AMBIGUITY_CUTOFF)
                    || mt2.rawFiducials[0].distToCamera > DISTANCE_CUTOFF;
            m_data.rejectMT2Update = rejectMT2Update;

            if (!rejectMT2Update) {
                double standard_devs = mt2.rawFiducials[0].distToCamera / DISTANCE_STDDEVS_SCALAR;
                m_swerve.setVisionMeasurementStdDevs(
                        VecBuilder.fill(standard_devs,
                                standard_devs, 9999999));
                m_swerve.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }

        }

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

    private boolean inFieldCheck(Pose2d pose) {
        boolean inLength = pose.getX() > 0 && pose.getX() < FieldConstants.fieldLength;
        boolean inWidth = pose.getY() > 0 && pose.getX() < FieldConstants.fieldWidth;
        return inLength && inWidth;
    }
}
