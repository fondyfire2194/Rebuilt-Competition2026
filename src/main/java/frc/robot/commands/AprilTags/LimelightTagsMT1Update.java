// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTags;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class LimelightTagsMT1Update extends Command {

    private final CommandSwerveDrivetrain m_swerve;
    private final LimelightVision m_llv;
    boolean rejectMT1Update;
    // private Pose2d lastPoseSeen = new Pose2d();
    private double xTolerance = .1;
    private double yTolerance = .1;
    private double rotTolerance = 5;
    // private int nearPoseCount;
    LimelightHelpers.PoseEstimate mt1 = new PoseEstimate();

    private int m_cameraIndex;

    public LimelightTagsMT1Update(LimelightVision llv, int cameraIndex, CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        m_llv = llv;
        m_cameraIndex = cameraIndex;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_llv.cameras[m_cameraIndex].camname);
        m_llv.mt1Pose[m_cameraIndex] = mt1.pose;

        m_llv.numberMT1TagsSeen[m_cameraIndex] = 0;
        for (int i = 0; i < m_llv.mt1TagIDsSeen.length - 1; i++) {
            m_llv.mt1TagIDsSeen[m_cameraIndex][i] = 0;
        }
        if (mt1.rawFiducials.length > 0) {
            m_llv.mt1ambiguity[m_cameraIndex] = mt1.rawFiducials[0].ambiguity;
            m_llv.mt1distToCamera[m_cameraIndex] = m_swerve.distanceLimelightToEstimator;
            m_llv.numberMT1TagsSeen[m_cameraIndex] = mt1.tagCount;
            m_swerve.distanceLimelightToEstimator = mt1.rawFiducials[0].distToCamera;
            m_llv.getMT1TagIDsSeen(m_cameraIndex, mt1.rawFiducials);
        }
        
        if (m_llv.useMT1) {
            if (poseNearLastPose(m_llv.mt1LastPoseSeen[m_cameraIndex], mt1.pose)) {
                m_llv.mt1NearPoseCount[m_cameraIndex]++;
            } else
                m_llv.mt1NearPoseCount[m_cameraIndex] = 0;
            m_llv.mt1LastPoseSeen[m_cameraIndex] = mt1.pose;

            rejectMT1Update = m_llv.mt1NearPoseCount[m_cameraIndex] < 10
                    || !inFieldCheck(mt1.pose)
                    || mt1.tagCount == 0
                    || mt1.tagCount == 1
                            && mt1.rawFiducials.length == 1
                            && mt1.rawFiducials[0].ambiguity > .7
                            && mt1.rawFiducials[0].distToCamera > 5;

            if (!rejectMT1Update) {
                m_llv.mt1PresetCount[m_cameraIndex]++;
                if (!m_llv.useMT2 && m_llv.mt1PresetCount[m_cameraIndex] >= m_llv.presetLimit) {
                    m_llv.useMT2 = true;
                    m_llv.useMT1 = false;
                }
                if (m_llv.useMT2)
                    m_llv.mt1PresetCount[m_cameraIndex] = 0;

                m_swerve.setVisionMeasurementStdDevs(
                        VecBuilder.fill(.7, .7, 1));
                m_swerve.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
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

    public int getIMUMode(String camName) {
        return (int) LimelightHelpers.getLimelightNTDouble(camName, "imu_set");
    }

    private boolean inFieldCheck(Pose2d pose) {
        boolean inLength = pose.getX() >= 0 && pose.getX() <= FieldConstants.fieldLength;
        boolean inWidth = pose.getY() >= 0 && pose.getY() <= FieldConstants.fieldWidth;
        return inLength && inWidth;
    }

    public boolean poseNearLastPose(Pose2d pose, Pose2d lastPose) {
        boolean xInTolerance = Math.abs(pose.getX() - lastPose.getX()) < xTolerance;
        boolean yInTolerance = Math.abs(pose.getY() - lastPose.getY()) < yTolerance;
        boolean rotInTolerance = Math
                .abs(pose.getRotation().getDegrees() - lastPose.getRotation().getDegrees()) < rotTolerance;
        return xInTolerance && yInTolerance && rotInTolerance;
    }
}
