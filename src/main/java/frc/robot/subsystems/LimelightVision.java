// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.CameraConstants.Cameras;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.IMUData;
import frc.robot.utils.SD;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public int frontCam = 0;
  public int leftCam = 1;
  public int rightCam = 2;

  public Cameras[] cameras = { CameraConstants.frontCamera, CameraConstants.leftCamera, CameraConstants.rightCamera };

  public String frontName = cameras[frontCam].camname;

  public String leftName = cameras[leftCam].camname;

  public String rightName = cameras[rightCam].camname;

  public boolean[] limelightExists;

  public boolean[] inhibitVision;

  public boolean showTelemetry = true;

  public Pose2d[] mt1Pose = { new Pose2d(), new Pose2d(), new Pose2d() };

  public int[] mt1TagCount;

  public double[] mt1Ambiguity;

  public double[] mt1DistToCamera;

  public double[] mt1TimeStampSeconds;

  StructPublisher<Pose2d> mt1FrontPosePublisher;
  StructPublisher<Pose2d> mt1LeftPosePublisher;
  StructPublisher<Pose2d> mt1RightPosePublisher;

  public Pose2d[] mt2Pose = { new Pose2d(), new Pose2d(), new Pose2d() };

  public double[] mt2ambiguity;

  public double[] mt2distToCamera;

  public int[] numberMT2Pose;

  public Pose2d[] acceptedPose = { new Pose2d(), new Pose2d(), new Pose2d() };
  public boolean mt1PoseSet;

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (FRONT, LEFT, RIGHT).
   * @return True if the limelight network table contains the key "tv"
   */
  public boolean isLimelightConnected(String camname) {
    return LimelightHelpers.getLimelightNTTable(camname).containsKey("tv");
  }

  public enum ImuMode {
    /**
     * Use external IMU yaw submitted via
     * {@link LimelightSettings#withRobotOrientation(Orientation3d)} for MT2
     * localization. The internal IMU is ignored entirely.
     */
    ExternalImu,
    /**
     * Use external IMU yaw submitted via
     * {@link LimelightSettings#withRobotOrientation(Orientation3d)} for MT2
     * localization. The internal IMU is synced with the external IMU.
     */
    SyncInternalImu,
    /**
     * Use internal IMU for MT2 localization. Ignores external IMU updates from
     * {@link LimelightSettings#withRobotOrientation(Orientation3d)}.
     */
    InternalImu,
    /**
     * Use internal IMU for MT2 localization. The internal IMU will utilize filtered
     * MT1 yaw estimates for continuous heading correction.
     */
    InternalImuMT1Assist,
    /**
     * Use internal IMU for MT2 localization. The internal IMU will utilize the
     * external IMU for continuous heading correction.
     */
    InternalImuExternalAssist
  }

  public LimelightVision() {

    mt1FrontPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic(frontName + " MT1FrontPose", Pose2d.struct).publish();
    mt1LeftPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic(leftName + " MT1LeftPose", Pose2d.struct).publish();
    mt1RightPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic(rightName + " MT1RightPose", Pose2d.struct).publish();

    setCamToRobotOffset(cameras[frontCam]);
    setCamToRobotOffset(cameras[leftCam]);
    setCamToRobotOffset(cameras[rightCam]);

  }

  @Override
  public void periodic() {

  }

  public Command setIMUModeCommand(int n) {
    return Commands.runOnce(() -> LimelightHelpers.SetIMUMode(frontName, n));
  }

  public void setCamToRobotOffset(Cameras cam) {
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
    return (int) LimelightHelpers.getLimelightNTDouble(camName, "imumode_set");
  }

  public String getIMUModeName(String camname) {
    int n = getIMUMode(camname);
    switch (n) {
      case 0:
        return ImuMode.ExternalImu.toString();
      case 1:
        return ImuMode.SyncInternalImu.toString();
      case 2:
        return ImuMode.InternalImu.toString();
      case 3:
        return ImuMode.InternalImuMT1Assist.toString();
      case 4:
        return ImuMode.InternalImuExternalAssist.toString();
      default:
        return "Unknown Mode";
    }

  }

  public double[] getLLHW(String camName) {
    return LimelightHelpers.getLimelightNTDoubleArray(camName, "hw");
  }

  public void setPipeline(String camName, int n) {
    LimelightHelpers.setPipelineIndex(camName, n);
  }

  public IMUData getIMUData() {
    return LimelightHelpers.getIMUData(frontName);
  }

  public double getIMUYaw() {
    return getIMUData().Yaw;
  }

  public double getIMUPitch() {
    return getIMUData().Pitch;
  }

  public double getIMURoll() {
    return getIMUData().Roll;
  }

  public double getIMUDataRobotYaw() {
    return getIMUData().robotYaw;
  }

  public void setAprilTagPipeline() {
    LimelightHelpers.setPipelineIndex(frontName, CameraConstants.apriltagPipeline);
    LimelightHelpers.setPipelineIndex(leftName, CameraConstants.apriltagPipeline);
    LimelightHelpers.setPipelineIndex(rightName, CameraConstants.apriltagPipeline);
  }

  public void setViewfinderPipeline() {
    if (CameraConstants.frontCamera.isLL4)
      LimelightHelpers.setPipelineIndex(frontName, CameraConstants.viewFinderPipeline);
    if (CameraConstants.leftCamera.isLL4)
      LimelightHelpers.setPipelineIndex(leftName, CameraConstants.viewFinderPipeline);
    if (CameraConstants.rightCamera.isLL4)
      LimelightHelpers.setPipelineIndex(rightName, CameraConstants.viewFinderPipeline);
  }

  private void showTelemetry() {

    SmartDashboard.putNumber("IMUMode#",
        getIMUMode(Constants.CameraConstants.frontCamera.camname));
    SmartDashboard.putString("IMUMode",
        getIMUModeName(Constants.CameraConstants.frontCamera.camname));

    SmartDashboard.putString("PipelineType",
        LimelightHelpers.getCurrentPipelineType(Constants.CameraConstants.frontCamera.camname));
    SmartDashboard.putNumber("Pipeline #",
        LimelightHelpers.getCurrentPipelineIndex(Constants.CameraConstants.frontCamera.camname));

    SD.sd2("IMUYaw", getIMUYaw());
    SD.sd2("IMUPitch", getIMUPitch());
    SD.sd2("IMURoll", getIMURoll());
    SD.sd2("IMUDataRobotYaw", getIMUDataRobotYaw());

    SmartDashboard.putBoolean("TagSeen",
        LimelightHelpers.getTV(Constants.CameraConstants.frontCamera.camname));
    SmartDashboard.putNumber("TagNumber",
        LimelightHelpers.getFiducialID(Constants.CameraConstants.frontCamera.camname));

    double[] vals = { 0, 0, 0, 0 };

    if (getLLHW(Constants.CameraConstants.frontCamera.camname).length > 0)
      vals = getLLHW(Constants.CameraConstants.frontCamera.camname);
    SD.sd1("LLTemperature", vals[0]);
    SD.sd1("LLCPU", vals[1]);
    SD.sd1("LLRam", vals[2]);
    SD.sd1("LLFPS", vals[3]);

  }

}
