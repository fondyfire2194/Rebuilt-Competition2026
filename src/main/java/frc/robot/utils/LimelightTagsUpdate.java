// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers.IMUData;

/** Add your docs here. */
public class LimelightTagsUpdate {

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

    StructPublisher<Pose2d> mt2PosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TagUpdate/MT2Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> mt1PosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TagUpdate/MT1Pose", Pose2d.struct).publish();

    public LimelightTagsUpdate(CameraConstants.CameraValues cam, CameraData data, CommandSwerveDrivetrain swerve) {
        m_cam = cam;
        m_swerve = swerve;
        m_data = data;
        m_data.inhibitVision = false;
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

    public void execute() {
        m_data.limeLightExists = LimelightHelpers.getLimelightNTTable(m_cam.camname).containsKey("tv");
        m_data.isActive = m_data.limeLightExists;

        if (m_data.isActive && LimelightHelpers.getTV(m_cam.camname)) {

            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cam.camname);
            m_data.mt1Pose = mt1.pose;
            m_data.numberMT1Pose = mt1.tagCount;

            setLLRobotOrientation();
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cam.camname);
            m_data.mt2Pose = mt2.pose;
            m_data.numberMT2Pose = mt2.tagCount;

            setUseMegatag2(m_data.m_useMegaTag2);

            if (m_useMegaTag2) {
                if (mt2.rawFiducials.length > 0)
                    m_swerve.distanceLimelightToEstimator = mt2.rawFiducials[0].distToCamera;
                m_data.distToCamera = m_swerve.distanceLimelightToEstimator;
                m_data.ambiguity = mt2.rawFiducials[0].ambiguity;
                rejectMT2Update = !m_data.inhibitVision && mt2.tagCount == 0
                        || Math.abs(m_swerve.getPigeon2().getAngularVelocityXDevice()
                                .getValueAsDouble()) > ROTATION_RATE_CUTOFF
                        || (mt2.tagCount == 1 && mt2.rawFiducials[0].ambiguity > AMBIGUITY_CUTOFF)
                        || mt2.rawFiducials[0].distToCamera > DISTANCE_CUTOFF;
                m_data.rejectMT2Update = rejectMT2Update;

                mt2PosePublisher.set(mt2.pose);// send to network tables

                if (!rejectMT2Update) {
                    double standard_devs = mt2.rawFiducials[0].distToCamera / DISTANCE_STDDEVS_SCALAR;
                    m_swerve.setVisionMeasurementStdDevs(
                            VecBuilder.fill(standard_devs,
                                    standard_devs, 9999999));
                    m_swerve.addVisionMeasurement(
                            mt2.pose,
                            mt2.timestampSeconds);
                }

            } else {

                rejectMT1Update = mt1.tagCount == 0
                        || mt1.tagCount == 1 && mt1.rawFiducials.length == 1 &&
                                mt1.rawFiducials[0].ambiguity > .7
                                && mt1.rawFiducials[0].distToCamera > 5;
                mt1PosePublisher.set(mt1.pose);
                m_data.rejectMT1Update = rejectMT1Update;
                if (!rejectMT1Update) {
                    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,
                            .7, 1));
                    m_swerve.addVisionMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                }
            }
        }
    }

    private double getLLToRobotPoseError(Pose2d ll) {
        Translation2d robtrans = m_swerve.getState().Pose.getTranslation();
        Translation2d lltrans = ll.getTranslation();
        return robtrans.getDistance(lltrans);
    }

    // private boolean inFieldCheck(Pose2d pose) {
    // boolean inLength = pose.getX() > 0 && pose.getX() <
    // FieldConstants.FIELD_LENGTH;
    // boolean inWidth = pose.getY() > 0 && pose.getX() <
    // FieldConstants.FIELD_WIDTH;

    // return inLength && inWidth;
    // }
}
