// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTags;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class PickAndSetPosetoMT1 extends Command {

    private final CommandSwerveDrivetrain m_swerve;
    private final LimelightVision m_llv;
    boolean rejectMT1Update;
    private final double AMBIGUITY_CUTOFF = 0.7;
    private final double DISTANCE_CUTOFF = 4.0;

    LimelightHelpers.PoseEstimate mt1Front = new PoseEstimate();
    LimelightHelpers.PoseEstimate mt1Left = new PoseEstimate();
    LimelightHelpers.PoseEstimate mt1Right = new PoseEstimate();

    String frontName = CameraConstants.frontCamera.camname;
    String leftName = CameraConstants.leftCamera.camname;
    String rightName = CameraConstants.rightCamera.camname;

    private boolean useLeftCameraResult;
    private boolean useRightCameraResult;
    private boolean useFrontCameraResult;

    int numberScans;
    final int scansAllowed = 10;

    public PickAndSetPosetoMT1(LimelightVision llv, CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        m_llv = llv;
    }

    @Override
    public void initialize() {
        useLeftCameraResult = false;
        useRightCameraResult = false;
        useFrontCameraResult = false;
    }

    @Override
    public void execute() {
        // choose MT1 pose between 3 camera MT1 results
        // camera already gives closest result

        if (m_llv.mt1TagCount[m_llv.leftCam] > 0 && m_llv.mt1DistToCamera[m_llv.leftCam] < DISTANCE_CUTOFF
                && m_llv.mt1Ambiguity[m_llv.leftCam] < AMBIGUITY_CUTOFF && inFieldCheck(m_llv.mt1Pose[m_llv.leftCam])) {
            useLeftCameraResult = true;
        }
        if (!useLeftCameraResult && m_llv.mt1TagCount[m_llv.rightCam] > 0
                && m_llv.mt1DistToCamera[m_llv.rightCam] < DISTANCE_CUTOFF
                && m_llv.mt1Ambiguity[m_llv.rightCam] < AMBIGUITY_CUTOFF
                && inFieldCheck(m_llv.mt1Pose[m_llv.rightCam])) {
            useRightCameraResult = true;
        }

        if (!useLeftCameraResult && !useRightCameraResult && m_llv.mt1TagCount[m_llv.frontCam] > 0
                && m_llv.mt1DistToCamera[m_llv.frontCam] < DISTANCE_CUTOFF
                && m_llv.mt1Ambiguity[m_llv.frontCam] < AMBIGUITY_CUTOFF
                && inFieldCheck(m_llv.mt1Pose[m_llv.frontCam])) {
            useFrontCameraResult = true;
        }

        if (useLeftCameraResult) {
            m_swerve.getPigeon2().setYaw(Degrees.of(m_llv.mt1Pose[m_llv.leftCam].getRotation().getDegrees()));
            m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, .1));
            m_swerve.addVisionMeasurement(
                    m_llv.acceptedPose[m_llv.leftCam],
                    m_llv.mt1TimeStampSeconds[m_llv.leftCam]);
        }

        if (useRightCameraResult) {
            m_swerve.getPigeon2().setYaw(Degrees.of(m_llv.mt1Pose[m_llv.rightCam].getRotation().getDegrees()));
            m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, .1));
            m_swerve.addVisionMeasurement(
                    m_llv.acceptedPose[m_llv.rightCam],
                    m_llv.mt1TimeStampSeconds[m_llv.rightCam]);
        }

        if (useFrontCameraResult) {
            m_swerve.getPigeon2().setYaw(Degrees.of(m_llv.mt1Pose[m_llv.frontCam].getRotation().getDegrees()));
            m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.1, .1, .1));
            m_swerve.addVisionMeasurement(
                    m_llv.acceptedPose[m_llv.frontCam],
                    m_llv.mt1TimeStampSeconds[m_llv.frontCam]);
        }
        numberScans++;

        m_llv.mt1PoseSet = useLeftCameraResult || useRightCameraResult || useFrontCameraResult;

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

    public int getIMUMode(String camName) {
        return (int) LimelightHelpers.getLimelightNTDouble(camName, "imu_set");
    }

    public void setAllLLRotationFromMT1(Pose2d pose) {
        LimelightHelpers.SetRobotOrientation(frontName,
                pose.getRotation().getDegrees(),
                m_swerve.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
        LimelightHelpers.SetRobotOrientation(leftName,
                pose.getRotation().getDegrees(),
                m_swerve.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
        LimelightHelpers.SetRobotOrientation(rightName,
                pose.getRotation().getDegrees(),
                m_swerve.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()

    }

    private boolean inFieldCheck(Pose2d pose) {
        boolean inLength = pose.getX() > 0 && pose.getX() < FieldConstants.fieldLength;
        boolean inWidth = pose.getY() > 0 && pose.getX() < FieldConstants.fieldWidth;
        return inLength && inWidth;
    }
}
