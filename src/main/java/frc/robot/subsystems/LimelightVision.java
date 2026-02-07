// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.IMUData;
import frc.robot.utils.SD;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public boolean limelightExistsFront;

  boolean allcamsok;

  public boolean limelightExistsLeft;

  public boolean inhibitFrontVision;
  public boolean inhibitRearVision;

  public String frontName = Constants.CameraConstants.frontCamera.camname;

  public String leftName = Constants.CameraConstants.leftCamera.camname;

  Optional<Pose3d> temp;

  public Pose2d frontAcceptedPose;

  public int frontAcceptedCount;

  public boolean frontRejectUpdate;

  public Pose2d rearAcceptedPose;
  public int rearAcceptedCount;

  StructPublisher<Pose2d> wpiBluePosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("LLV/WPIBluePose", Pose2d.struct).publish();

  public boolean showTelemetry=true;

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
    if (CameraConstants.frontCamera.isUsed) {
      setCamToRobotOffset(Constants.CameraConstants.frontCamera);
      LimelightHelpers.SetIMUMode(frontName, ImuMode.ExternalImu.ordinal());
    }
    if (CameraConstants.leftCamera.isUsed) {
      setCamToRobotOffset(Constants.CameraConstants.leftCamera);
      LimelightHelpers.SetIMUMode(leftName, ImuMode.ExternalImu.ordinal());
    }
  }

  @Override
  public void periodic() {

    if (RobotBase.isReal()) {
      limelightExistsFront = isLimelightConnected(CameraConstants.frontCamera.camname);
      limelightExistsLeft = isLimelightConnected(CameraConstants.leftCamera.camname);
      CameraConstants.frontCamera.isActive = !inhibitFrontVision && limelightExistsFront;
      CameraConstants.leftCamera.isActive = !inhibitRearVision && limelightExistsLeft;

      allcamsok = Constants.CameraConstants.frontCamera.isUsed && limelightExistsFront
          && Constants.CameraConstants.leftCamera.isUsed && limelightExistsLeft;

      if (limelightExistsFront && LimelightHelpers.getTV(frontName))
        wpiBluePosePublisher.set(LimelightHelpers.getBotPose3d_wpiBlue(frontName).toPose2d());

      if (limelightExistsLeft && LimelightHelpers.getTV(leftName))
        wpiBluePosePublisher.set(LimelightHelpers.getBotPose3d_wpiBlue(leftName).toPose2d());

      if (showTelemetry) {
        showTelemetry();
      }

    }
  }

  public Command setIMUModeCommand(int n) {
    return Commands.runOnce(() -> LimelightHelpers.SetIMUMode(frontName, n));
  }

  public void setCamToRobotOffset(Constants.CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
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