// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.CameraData;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.IMUData;
import frc.robot.utils.SD;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public String frontName = Constants.CameraConstants.frontCamera.camname;

  public boolean limelightExistsFront;

  public boolean inhibitFrontVision;

  public Pose2d frontAcceptedPose;

  public int frontAcceptedCount;

  public boolean frontRejectUpdate;

  public String leftName = Constants.CameraConstants.leftCamera.camname;

  public boolean limelightExistsLeft;

  public boolean inhibitLeftVision;

  public Pose2d leftAcceptedPose;

  public int leftAcceptedCount;

  public boolean leftRejectUpdate;

  public String rightName = Constants.CameraConstants.rightCamera.camname;

  public boolean limelightExistsRight;

  public boolean inhibitRightVision;

  public Pose2d rightAcceptedPose;

  public int rightAcceptedCount;

  public boolean rightRejectUpdate;

  Optional<Pose3d> temp;

  public boolean showTelemetry = true;

  
    public CameraData frontData = new CameraData(CameraConstants.frontCamera.camname);
    public CameraData leftData = new CameraData(CameraConstants.leftCamera.camname);
    public CameraData rightData = new CameraData(CameraConstants.rightCamera.camname);


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

  }

  @Override
  public void periodic() {

    if (showTelemetry) {
      showTelemetry();
    }

  }

  public Command setIMUModeCommand(int n) {
    return Commands.runOnce(() -> LimelightHelpers.SetIMUMode(frontName, n));
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
