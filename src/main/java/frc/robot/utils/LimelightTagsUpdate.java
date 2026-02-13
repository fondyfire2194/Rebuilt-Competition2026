// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
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

    // private StructArrayPublisher<AprilTag> arrayPublisher =
    // NetworkTableInstance.getDefault()
    // .getStructArrayTopic("AprilTags", new AprilTagStruct()).publish();
    private StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("P3d", Pose3d.struct).publish();
    Pose3d[] camPoses = new Pose3d[4];

    StructPublisher<Pose2d> mt2PosePublisher;

    StructPublisher<Pose2d> mt1PosePublisher;

    public LimelightTagsUpdate(CameraConstants.CameraValues cam, CameraData data, CommandSwerveDrivetrain swerve) {
        m_cam = cam;
        m_swerve = swerve;
        m_data = data;
        m_data.inhibitVision = false;
        setCamToRobotOffset(m_cam);
        if (m_cam.camname == CameraConstants.frontCamera.camname)
            CameraConstants.frontCamPosePublisher.set(CameraConstants.frontCamera.camPose.toPose2d());
        if (m_cam.camname == CameraConstants.leftCamera.camname)
            CameraConstants.leftCamPosePublisher.set(CameraConstants.leftCamera.camPose.toPose2d());
        if (m_cam.camname == CameraConstants.rightCamera.camname)
            CameraConstants.rightCamPosePublisher.set(CameraConstants.rightCamera.camPose.toPose2d());

        mt2PosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("MT2Pose " + m_cam.camname, Pose2d.struct).publish();
        mt1PosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("MT1Pose " + m_cam.camname, Pose2d.struct).publish();

        if (m_cam.camname == "limelight-front") {

            camPoses[0] = new Pose3d(
                    Units.inchesToMeters(9.5), // front of robot
                    Units.inchesToMeters(0), // on LR center
                    Units.inchesToMeters(8), // high
                    new Rotation3d(
                            Units.degreesToRadians(0), // no roll
                            Units.degreesToRadians(120), // angled up
                            Units.degreesToRadians(0))); // facing forward

            camPoses[1] = new Pose3d(
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(8),
                    new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(120),
                            Units.degreesToRadians(100)));

            camPoses[2] = new Pose3d(
                    Units.inchesToMeters(-0),
                    Units.inchesToMeters(-10),
                    Units.inchesToMeters(8),
                    new Rotation3d(
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(120),
                            Units.degreesToRadians(-100)));

            camPoses[3] = new Pose3d(
                    Units.inchesToMeters(-0),
                    Units.inchesToMeters(-10),
                    Units.inchesToMeters(8),
                    new Rotation3d(
                            Units.degreesToRadians(-16.875),
                            Units.degreesToRadians(0),
                            Units.degreesToRadians(-4.7)));

            arrayPublisher.accept(camPoses);
        }
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

    public void execute() {
        m_data.limeLightExists = LimelightHelpers.getLimelightNTTable(m_cam.camname).containsKey("tv");
        m_data.isActive = m_data.limeLightExists;

        if (m_data.isActive && LimelightHelpers.getTV(m_cam.camname)) {

            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cam.camname);
            m_data.mt1Pose = mt1.pose;
            mt1PosePublisher.set(mt1.pose);
            m_data.numberMT1Pose = mt1.tagCount;

            setLLRobotOrientation();
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cam.camname);
            m_data.mt2Pose = mt2.pose;
            mt2PosePublisher.set(mt2.pose);// send to network tables
            m_data.numberMT2Pose = mt2.tagCount;

            setUseMegatag2(m_data.m_useMegaTag2);

            if (m_useMegaTag2) {
                if (mt2.rawFiducials.length > 0)
                    m_swerve.distanceLimelightToEstimator = mt2.rawFiducials[0].distToCamera;
                m_data.distToCamera = m_swerve.distanceLimelightToEstimator;
                m_data.MT2ambiguity = mt2.rawFiducials[0].ambiguity;
                rejectMT2Update = !m_data.inhibitVision && mt2.tagCount == 0
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

            } else {

                rejectMT1Update = mt1.tagCount == 0
                        || mt1.tagCount == 1 && mt1.rawFiducials.length == 1
                                && mt1.rawFiducials[0].ambiguity > .7
                                && mt1.rawFiducials[0].distToCamera > 5;

                m_data.rejectMT1Update = rejectMT1Update;
                if (!rejectMT1Update) {
                    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 1));
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
