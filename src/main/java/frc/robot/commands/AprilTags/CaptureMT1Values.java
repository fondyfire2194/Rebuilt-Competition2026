// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTags;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class CaptureMT1Values extends Command {

    private final LimelightVision m_llv;
    boolean rejectMT1Update;

    LimelightHelpers.PoseEstimate mt1Front = new PoseEstimate();
    LimelightHelpers.PoseEstimate mt1Left = new PoseEstimate();
    LimelightHelpers.PoseEstimate mt1Right = new PoseEstimate();
    int numberScans;
    final int scansAllowed = 10;

    public CaptureMT1Values(LimelightVision llv) {
        m_llv = llv;
    }

    @Override
    public void initialize() {

        numberScans = 0;
    }

    @Override
    public void execute() {

        if (LimelightHelpers.getTV(m_llv.leftName)) {
            mt1Left = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_llv.leftName);
            m_llv.mt1Pose[m_llv.leftCam] = mt1Left.pose;
            m_llv.mt1TagCount[m_llv.leftCam] = mt1Left.tagCount;
            if (mt1Left.rawFiducials.length > 0) {
                m_llv.mt1Ambiguity[m_llv.leftCam] = mt1Left.rawFiducials[0].ambiguity;
                m_llv.mt1DistToCamera[m_llv.leftCam] = mt1Left.rawFiducials[0].distToCamera;
                m_llv.mt1TimeStampSeconds[m_llv.leftCam] = mt1Left.timestampSeconds;
                m_llv.getMT1TagsSeen(1, mt1Left.rawFiducials);
            }
        }
        if (LimelightHelpers.getTV(m_llv.rightName)) {
            mt1Right = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_llv.rightName);
            m_llv.mt1Pose[m_llv.rightCam] = mt1Right.pose;
            m_llv.mt1TagCount[m_llv.rightCam] = mt1Right.tagCount;
            m_llv.mt1TimeStampSeconds[m_llv.rightCam] = mt1Right.timestampSeconds;
            if (mt1Right.rawFiducials.length > 0) {
                m_llv.mt1Ambiguity[m_llv.rightCam] = mt1Right.rawFiducials[0].ambiguity;
                m_llv.mt1DistToCamera[m_llv.rightCam] = mt1Right.rawFiducials[0].distToCamera;
                m_llv.getMT1TagsSeen(2, mt1Left.rawFiducials);
            }
        }

        if (LimelightHelpers.getTV(m_llv.frontName)) {
            mt1Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_llv.frontName);
            m_llv.mt1Pose[m_llv.frontCam] = mt1Front.pose;
            m_llv.mt1TagCount[m_llv.frontCam] = mt1Front.tagCount;
            m_llv.mt1TimeStampSeconds[m_llv.frontCam] = mt1Front.timestampSeconds;
            if (mt1Front.rawFiducials.length > 0) {
                m_llv.mt1Ambiguity[m_llv.frontCam] = mt1Front.rawFiducials[0].ambiguity;
                m_llv.mt1DistToCamera[m_llv.frontCam] = mt1Front.rawFiducials[0].distToCamera;
                m_llv.getMT1TagsSeen(0, mt1Left.rawFiducials);
            }
        }

        numberScans++;

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return numberScans >= scansAllowed;
    }

}
