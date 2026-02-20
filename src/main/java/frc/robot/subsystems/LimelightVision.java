// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.CameraConstants.Cameras;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.IMUData;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public int frontCam = 0;
  public int leftCam = 1;
  public int rightCam = 2;

  public Cameras[] cameras = new Cameras[3];

  public String frontName;

  public String leftName;

  public String rightName;

  public boolean[] limelightExists;

  public boolean[] inhibitVision;

  public Pose2d[] mt1Pose = { new Pose2d(), new Pose2d(), new Pose2d() };

  public int[] mt1TagCount;

  public double[][] mt1TagsSeen = new double[3][5];

  public double[][] mt2TagsSeen = new double[3][5];

  public double[] mt1Ambiguity;

  public double[] mt1DistToCamera;

  public double[] mt1TimeStampSeconds;

  StructPublisher<Pose2d> mt1FrontPosePublisher;
  StructPublisher<Pose2d> mt1LeftPosePublisher;
  StructPublisher<Pose2d> mt1RightPosePublisher;

  StructPublisher<Pose2d> mt2FrontPosePublisher;
  StructPublisher<Pose2d> mt2LeftPosePublisher;
  StructPublisher<Pose2d> mt2RightPosePublisher;

  public Pose2d[] mt2Pose = { new Pose2d(), new Pose2d(), new Pose2d() };

  public double[] mt2ambiguity;

  public double[] mt2distToCamera;

  public int[] numberMT2Pose;

  public Pose2d[] acceptedPose = { new Pose2d(), new Pose2d(), new Pose2d() };

  public boolean mt1PoseSet;

  public boolean useMT2;

  private boolean showData;

  double[] vals = { 0, 0, 0, 0, 0, 0, 0, 0 };

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

  public LimelightVision(boolean showData) {
    this.showData = showData;
    cameras[0] = CameraConstants.frontCamera;
    cameras[1] = CameraConstants.leftCamera;
    cameras[2] = CameraConstants.rightCamera;

    frontName = cameras[frontCam].camname;

    leftName = cameras[leftCam].camname;

    rightName = cameras[rightCam].camname;

    mt1FrontPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("LimelightPoses/" + frontName + " MT1FrontPose", Pose2d.struct).publish();
    mt1LeftPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("LimelightPoses/" + leftName + " MT1LeftPose", Pose2d.struct).publish();
    mt1RightPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("LimelightPoses/" + rightName + " MT1RightPose", Pose2d.struct).publish();

    mt2FrontPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("LimelightPoses/" + frontName + " MT2FrontPose", Pose2d.struct).publish();
    mt2LeftPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("LimelightPoses/" + leftName + " MT2LeftPose", Pose2d.struct).publish();
    mt2RightPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("LimelightPoses/" + rightName + " MT2RightPose", Pose2d.struct).publish();

    setCamToRobotOffset(cameras[frontCam]);
    setCamToRobotOffset(cameras[leftCam]);
    setCamToRobotOffset(cameras[rightCam]);

    if (showData)
      SmartDashboard.putData(this);

  }

  public Command startMT2UpdatesCommand() {
    return Commands.runOnce(() -> useMT2 = true);
  }

  @Override
  public void periodic() {
    mt1FrontPosePublisher.accept(mt1Pose[frontCam]);
    mt1LeftPosePublisher.accept(mt1Pose[leftCam]);
    mt1RightPosePublisher.accept(mt1Pose[rightCam]);

    mt2FrontPosePublisher.accept(mt2Pose[frontCam]);
    mt2LeftPosePublisher.accept(mt2Pose[leftCam]);
    mt2RightPosePublisher.accept(mt2Pose[rightCam]);

    if (showData) {
      if (getLLHW(Constants.CameraConstants.frontCamera.camname).length > 0)
        vals = getLLHW(Constants.CameraConstants.frontCamera.camname);
  }

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

  public void getMT1TagsSeen(int cameraPointer, RawFiducial[] rawFiducial) {
    for (int i = 0; i < mt1TagsSeen.length; i++) {
      mt1TagsSeen[cameraPointer][i] = 0;
    }
    // for (int i = 0; i < rawFiducial.length; i++) {
    // mt1TagsSeen[cameraPointer][i] = rawFiducial[i].id;
    // }
  }

  public void getMT2TagsSeen(int cameraPointer, RawFiducial[] rawFiducial) {
    for (int i = 0; i < mt1TagsSeen.length; i++) {
      mt2TagsSeen[cameraPointer][i] = 0;
    }
    // for (int i = 0; i < rawFiducial.length; i++) {
    // mt1TagsSeen[cameraPointer][i] = rawFiducial[i].id;
    // }
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

  private void initSendable(SendableBuilder builder, int cameraIndex) {
    builder.addDoubleProperty(cameras[cameraIndex].camname + " Pipeline",
        () -> LimelightHelpers.getCurrentPipelineIndex(cameras[cameraIndex].camname), null);
    builder.addStringProperty(cameras[cameraIndex].camname + " Pipeline Type",
        () -> LimelightHelpers.getCurrentPipelineType(cameras[cameraIndex].camname), null);
    builder.addBooleanProperty(cameras[cameraIndex].camname + " Tag Seen",
        () -> LimelightHelpers.getTV(cameras[cameraIndex].camname), null);
    builder.addDoubleArrayProperty(cameras[cameraIndex].camname + " MT1TagsSeen", () -> mt1TagsSeen[cameraIndex], null);
    builder.addDoubleArrayProperty(cameras[cameraIndex].camname + " MT2TagsSeen", () -> mt2TagsSeen[cameraIndex], null);

  }

  private void initSendableLL4(SendableBuilder builder, String name, int cameraIndex) {
    builder.addDoubleProperty(name + " Pipeline", () -> LimelightHelpers.getCurrentPipelineIndex(name), null);
    builder.addStringProperty(name + " Pipeline Type", () -> LimelightHelpers.getCurrentPipelineType(name), null);
    builder.addBooleanProperty(name + " Tag Seen", () -> LimelightHelpers.getTV(name), null);
    builder.addDoubleProperty(name + " IMU Yaw", () -> getIMUYaw(), null);
    builder.addDoubleProperty(name + " IMU Robot Yaw", () -> getIMUDataRobotYaw(), null);
    builder.addDoubleProperty(name + " IMU Pitch", () -> getIMUPitch(), null);
    builder.addDoubleProperty(name + " IMU Roll", () -> getIMURoll(), null);
    builder.addStringProperty(name + " IMU Mode", () -> getIMUModeName(name), null);
    builder.addDoubleProperty(name + " IMU Yaw", () -> getIMUYaw(), null);
    builder.addDoubleArrayProperty(name + " MT1TagsSeen", () -> mt1TagsSeen[cameraIndex], null);
    builder.addDoubleArrayProperty(name + " MT2TagsSeen", () -> mt2TagsSeen[cameraIndex], null);

    builder.addDoubleProperty(name + "  Temperature", () -> vals[0], null);
    builder.addDoubleProperty(name + " CPU", () -> vals[1], null);
    builder.addDoubleProperty(name + " Ram", () -> vals[2], null);
    builder.addDoubleProperty(name + " FPS", () -> vals[2], null);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    initSendableLL4(builder, frontName, 0);
    initSendable(builder, 1);
    initSendable(builder, 2);

  }
}
