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
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class LimelightTagsMT1Update extends Command {

    private final CommandSwerveDrivetrain m_swerve;
    private final CameraConstants.CameraValues m_cam;
    boolean rejectMT1Update;
    CameraData m_data;
    private final double AMBIGUITY_CUTOFF = 0.7;
    private final double DISTANCE_CUTOFF = 4.0;
    private final double DISTANCE_STDDEVS_SCALAR = 2;
    private final double ROTATION_RATE_CUTOFF = 720;
    LimelightHelpers.PoseEstimate mt1 = new PoseEstimate();

    StructPublisher<Pose2d> mt1PosePublisher;

    private boolean showData = true;

    public LimelightTagsMT1Update(CameraConstants.CameraValues cam,CameraData data,
            CommandSwerveDrivetrain swerve) {
        m_cam = cam;
        m_swerve = swerve;
        m_data.inhibitVision = false;
        setCamToRobotOffset(m_cam);
        if (m_cam.camname == "limelight-front") {
            CameraConstants.arrayPublisher.accept(CameraConstants.camPoses);
        }

        mt1PosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(m_cam.camname + " MT1Pose", Pose2d.struct).publish();

    }

    @Override
    public void initialize() {
    
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

    public int getIMUMode(String camName) {
        return (int) LimelightHelpers.getLimelightNTDouble(camName, "imu_set");
    }

    public void setLLRotationFromMT1(String camName) {
        LimelightHelpers.SetRobotOrientation(m_cam.camname,
                m_data.mt1Pose.getRotation().getDegrees(),
                m_swerve.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
        m_data.setMT2toMT1Rotation = false;
        // m_data.m_useMegaTag2 = true;
    }

    @Override
    public void execute() {

        m_data.limeLightExists = LimelightHelpers.getLimelightNTTable(m_cam.camname).containsKey("tv");
        m_data.isActive = m_data.limeLightExists;

        if (m_data.isActive && LimelightHelpers.getTV(m_cam.camname)) {
            mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cam.camname);
            m_data.mt1Pose = mt1.pose;
            m_data.MT1tagCount = mt1.tagCount;
            if (mt1.rawFiducials.length > 0) {
                m_data.MT1ambiguity = mt1.rawFiducials[0].ambiguity;
                m_data.MT1distToCamera = mt1.rawFiducials[0].distToCamera;
                if (showData) {
                    mt1PosePublisher.set(mt1.pose);
                    SmartDashboard.putNumber(m_cam.camname + " MT1 Tag Count", mt1.tagCount);
                    SmartDashboard.putNumber(m_cam.camname + " MT1 Abiguity", m_data.MT1ambiguity);
                    SmartDashboard.putNumber(m_cam.camname + " MT1 Dist To Cam", m_data.MT1distToCamera);
                    SmartDashboard.putNumber(m_cam.camname + " MT1 Rotation To Cam",
                            m_data.mt1Pose.getRotation().getDegrees());
                    SmartDashboard.putBoolean(m_cam.camname + " MT1RejectUpdate", rejectMT1Update);
                }
            }
        }

        if (m_data.setMT2toMT1Rotation) {
            setLLRotationFromMT1(m_cam.camname);
            m_data.orientationSet = true;
        }

        rejectMT1Update = m_data.inhibitVision || mt1.tagCount == 0 || !inFieldCheck(m_data.mt1Pose)
                || mt1.tagCount == 1 && mt1.rawFiducials[0].ambiguity > AMBIGUITY_CUTOFF
                || Math.abs(m_swerve.getPigeon2().getAngularVelocityXDevice()
                        .getValueAsDouble()) > ROTATION_RATE_CUTOFF
                || mt1.rawFiducials[0].distToCamera > DISTANCE_CUTOFF;

        m_data.rejectMT1Update = rejectMT1Update;

        if (!rejectMT1Update) {
            m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 1));
            m_swerve.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
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
