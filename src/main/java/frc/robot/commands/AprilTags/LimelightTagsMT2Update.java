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
public class LimelightTagsMT2Update extends Command {

    private final CommandSwerveDrivetrain m_swerve;
    private final LimelightVision m_llv;
    boolean rejectMT2Update;
    private final double AMBIGUITY_CUTOFF = 0.7;
    private final double DISTANCE_CUTOFF = 4.0;
    private final double DISTANCE_STDDEVS_SCALAR = 2;
    private final double ROTATION_RATE_CUTOFF = 720;

    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();

    private int m_cameraIndex;

    public LimelightTagsMT2Update(LimelightVision llv, int cameraIndex, CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        m_llv = llv;
        m_cameraIndex = cameraIndex;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_llv.useMT2) {
            LimelightHelpers.SetRobotOrientation(m_llv.cameras[m_cameraIndex].camname,
                    // m_swerve.getState().Pose.getRotation().getDegrees(),
                    m_swerve.getPigeon2().getYaw().getValueAsDouble();
                    m_swerve.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0,
                    0);
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_llv.cameras[m_cameraIndex].camname);
            m_llv.mt2Pose[m_cameraIndex] = mt2.pose;

            if (mt2.rawFiducials.length > 0) {
                m_llv.mt2ambiguity[m_cameraIndex] = mt2.rawFiducials[0].ambiguity;
                m_llv.mt2distToCamera[m_cameraIndex] = m_swerve.distanceLimelightToEstimator;
                m_llv.numberMT2Pose[m_cameraIndex] = mt2.tagCount;
                m_swerve.distanceLimelightToEstimator = mt2.rawFiducials[0].distToCamera;
                m_llv.getMT2TagsSeen(m_cameraIndex, mt2.rawFiducials);
            }

            rejectMT2Update = mt2.tagCount == 0 || !inFieldCheck(m_llv.mt2Pose[m_cameraIndex])
                    || Math.abs(m_swerve.getPigeon2().getAngularVelocityXDevice()
                            .getValueAsDouble()) > ROTATION_RATE_CUTOFF
                    || (mt2.tagCount == 1 && mt2.rawFiducials[0].ambiguity > AMBIGUITY_CUTOFF)
                    || mt2.rawFiducials[0].distToCamera > DISTANCE_CUTOFF;

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

    public int getIMUMode(String camName) {
        return (int) LimelightHelpers.getLimelightNTDouble(camName, "imu_set");
    }

    private boolean inFieldCheck(Pose2d pose) {
        boolean inLength = pose.getX() > 0 && pose.getX() < FieldConstants.fieldLength;
        boolean inWidth = pose.getY() > 0 && pose.getX() < FieldConstants.fieldWidth;
        return inLength && inWidth;
    }
}
