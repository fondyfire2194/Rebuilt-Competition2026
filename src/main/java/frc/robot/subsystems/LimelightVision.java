// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public Cameras[] cameras = new Cameras[4];

  public String frontName;

  public String leftName;

  public String rightName;

  public boolean frontConnected;
  public boolean leftConnected;
  public boolean rightConnected;

  public double lastFrontHeartbeat;
  public double lastLeftHeartbeat;
  public double lastRightHeartbeat;

  public int numberOfAprilTagCameras = 1;

  public int numberTagsAllowed = 5;

  public boolean[] inhibitVision = new boolean[numberTagsAllowed];

  public Pose2d[] mt1Pose = { new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d() };

  public int[] mt1TagCount = new int[numberTagsAllowed];

  public double[][] mt1TagIDsSeen = new double[numberOfAprilTagCameras][numberTagsAllowed];

  public double[][] mt2TagIDsSeen = new double[numberOfAprilTagCameras][numberTagsAllowed];

  public double[] mt1DistToCamera = new double[numberTagsAllowed];

  public double[] mt1TimeStampSeconds = new double[numberTagsAllowed];

  public Pose2d[] mt2LastPoseSeen = { new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d() };

  public int[] mt2NearPoseCount = new int[numberOfAprilTagCameras];

  public int[] mt1PresetCount = new int[numberOfAprilTagCameras];

  public int presetLimit = 50;

  public Pose2d[] mt2Pose = { new Pose2d(), new Pose2d() };

  public double[] mt2ambiguity = new double[numberOfAprilTagCameras];

  public double[] mt2distToCamera = new double[numberOfAprilTagCameras];

  public double[] numberMT2TagsSeen = new double[numberOfAprilTagCameras];

  public double[] mt1ambiguity = new double[numberOfAprilTagCameras];

  public boolean[] mt2RejectUpdate = new boolean[numberOfAprilTagCameras];

  public double[] mt1distToCamera = new double[numberOfAprilTagCameras];

  public double[] numberMT1TagsSeen = new double[numberOfAprilTagCameras];

  public Pose2d[] acceptedPose = { new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d() };

  public boolean mt1PoseSet;

  public boolean useMT2;

  public boolean useMT1;

  private boolean logData;

  double[] vals = new double[8];

  double[] lastHeartbeat = new double[numberOfAprilTagCameras];

  public double tagID;

  Alert frontCameraDisconnected = new Alert("Front Camera Disconnected",
      AlertType.kError);
  Alert leftCameraDisconnected = new Alert("Left Camera Disconnected",
      AlertType.kError);
  Alert rightCameraDisconnected = new Alert("Right Camera Disconnected",
      AlertType.kError);

  private int logStep;

  public static final Transform3d RIGHT_BACK_CAMERA_POSITION = new Transform3d(
      Inches.of(-9.607),
      Inches.of(-13.028588),
      Inches.of(8.125806),
      new Rotation3d(Degrees.of(0), Degrees.of(-23), Degrees.of(-165)));

  public static Pose3d RBCP = new Pose3d(
      Units.inchesToMeters(-9.607),
      Units.inchesToMeters(-13.028),
      Units.inchesToMeters(8.12),
      new Rotation3d(Degrees.of(0), Degrees.of(-23), Degrees.of(-165)));

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

  public LimelightVision(boolean logData) {
    this.logData = logData;

    cameras[0] = CameraConstants.frontCamera;
    cameras[1] = CameraConstants.leftCamera;
    cameras[2] = CameraConstants.rightCamera;

    frontName = cameras[frontCam].camname;
    leftName = cameras[leftCam].camname;
    rightName = cameras[rightCam].camname;

    setCamToRobotOffset(cameras[frontCam]);

    if (CameraConstants.leftCamera.isUsed)
      setCamToRobotOffset(cameras[leftCam]);

    if (CameraConstants.rightCamera.isUsed)
      setCamToRobotOffset(cameras[rightCam]);

    frontCameraDisconnected.set(!frontConnected);
    // leftCameraDisconnected.set(!leftConnected);
    // rightCameraDisconnected.set(!rightConnected);

  }

  public double getCameraHeartbeat(String camName) {
    return LimelightHelpers.getHeartbeat(camName);
  }

  @Override
  public void periodic() {
    if (logData) {
      logStep++;

      switch (logStep) {

        case 0:

          DogLog.log("LLV_FrontCamMT1Pose", mt1Pose[frontCam]);
          DogLog.log("LLV_FrontCamMT2Pose", mt2Pose[frontCam]);
          DogLog.log("LLV_FrontCamMT2TagsSeen", mt2TagIDsSeen[frontCam]);
          DogLog.log("LLV_FrontCam # MT2TagsSeen", numberMT2TagsSeen[frontCam]);
          DogLog.log("LLV_FrontCamMT1TagsSeen", mt1TagIDsSeen[frontCam]);
          DogLog.log("LLV_FrontCam # MT1TagsSeen", numberMT1TagsSeen[frontCam]);

          break;

        case 1:
          DogLog.log("LLV_FrontCamDist", mt2distToCamera);
          DogLog.log("LLV_FrontCamAmbiguity", mt2ambiguity[frontCam]);
          DogLog.log("LLV_FrontCamDReject", mt2RejectUpdate[frontCam]);

          DogLog.log("LLV_FrontCamUseMT1", useMT1);
          DogLog.log("LLV_FrontCamUseMT2", useMT2);
          DogLog.log("LLV_FrontPipeline#", LimelightHelpers.getCurrentPipelineIndex(frontName));

          break;

        case 2:
          if (CameraConstants.leftCamera.isUsed) {
            DogLog.log("LLV_LeftCamMT2Pose", mt2Pose[leftCam]);
            DogLog.log("LLV_LeftCamMT2TagsSeen", mt2TagIDsSeen[leftCam]);
            DogLog.log("LLV_LeftCam # MT2TagsSeen", numberMT2TagsSeen[leftCam]);
            DogLog.log("LLV_LeftPipeline#", LimelightHelpers.getCurrentPipelineIndex(leftName));
          }
          break;

        case 3:
          if (CameraConstants.rightCamera.isUsed) {
            DogLog.log("LLV_RightCamMT2Pose", mt2Pose[rightCam]);
            DogLog.log("LLV_RightCamMT2TagsSeen", mt2TagIDsSeen[rightCam]);
            DogLog.log("LLV_RightCam # MT2TagsSeen", numberMT2TagsSeen[rightCam]);
            DogLog.log("LLV_RightPipeline#", LimelightHelpers.getCurrentPipelineIndex(rightName));
          }
          break;

        case 4:

          double leftHeartbeat = 0;
          double rightHeartbeat = 0;
          if (logData) {
            double frontHeartbeat = getCameraHeartbeat(frontName);
            if (CameraConstants.leftCamera.isUsed) {
              leftHeartbeat = getCameraHeartbeat(leftName);
              leftConnected = leftHeartbeat != lastLeftHeartbeat;
              lastLeftHeartbeat = leftHeartbeat;
              if (leftConnected && leftCameraDisconnected.get())
                leftCameraDisconnected.close();
            }
            if (CameraConstants.rightCamera.isUsed) {
              rightHeartbeat = getCameraHeartbeat(rightName);
              rightConnected = rightHeartbeat != lastRightHeartbeat;
              lastRightHeartbeat = rightHeartbeat;
              if (rightConnected && rightCameraDisconnected.get())
                rightCameraDisconnected.close();
            }
            
            frontConnected = frontHeartbeat != lastFrontHeartbeat;
            lastFrontHeartbeat = frontHeartbeat;

            if (frontConnected && frontCameraDisconnected.get())
              frontCameraDisconnected.close();

          }
          break;

        case 5:
          logStep = -1;
          break;

        default:
          logStep = -1;
          break;
      }
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
        Units.radiansToDegrees(cam.camPose.getRotation().getX()),
        Units.radiansToDegrees(cam.camPose.getRotation().getY()),
        Units.radiansToDegrees(cam.camPose.getRotation().getZ()));
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

  public void setDefaultLLPipelines() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontCamera.camname,
        CameraConstants.apriltagPipeline);
    if (CameraConstants.leftCamera.isUsed)
      LimelightHelpers.setPipelineIndex(CameraConstants.leftCamera.camname,
          CameraConstants.apriltagPipeline);
    if (CameraConstants.rightCamera.isUsed)
      LimelightHelpers.setPipelineIndex(CameraConstants.rightCamera.camname,
          CameraConstants.apriltagPipeline);
  }

  public void setViewFinderLLPipelines() {
    LimelightHelpers.setPipelineIndex(CameraConstants.leftCamera.camname,
        CameraConstants.viewFinderPipeline);
    LimelightHelpers.setPipelineIndex(CameraConstants.rightCamera.camname,
        CameraConstants.viewFinderPipeline);

  }

  public boolean getFrontCamSeesHubTags() {
    return LimelightHelpers.getFiducialID(frontName) == 9
        || LimelightHelpers.getFiducialID(frontName) == 10
        || LimelightHelpers.getFiducialID(frontName) == 25
        || LimelightHelpers.getFiducialID(frontName) == 26;
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

  public void getMT1TagIDsSeen(int cameraPointer, RawFiducial[] rawFiducial) {
    for (int i = 0; i < mt1TagIDsSeen.length; i++) {
      mt1TagIDsSeen[cameraPointer][i] = 0;
    }
    if (rawFiducial != null) {
      for (int i = 0; i < rawFiducial.length; i++) {
        mt1TagIDsSeen[cameraPointer][i] = rawFiducial[i].id;
      }
    }
  }

  public void getMT2TagIDsSeen(int cameraPointer, RawFiducial[] rawFiducial) {
    for (int i = 0; i < mt2TagIDsSeen.length - 1; i++) {
      mt2TagIDsSeen[cameraPointer][i] = 0;
    }
    if (rawFiducial != null) {
      for (int i = 0; i < rawFiducial.length; i++) {
        mt2TagIDsSeen[cameraPointer][i] = rawFiducial[i].id;
      }
    }
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

  /**
   * 
   * // Basic filtering thresholds
   * public static final double MAX_AMBIGUITY = 0.3;
   * public static final double MAX_Z_ERROR = 0.75;
   * 
   * // Standard deviation baselines, for 1 meter distance and 1 tag
   * // (Adjusted automatically based on distance and # of tags)
   * public static final double LINEAR_STDDEV_BASELINE = 0.02; // Meters
   * public static final double ANGULAR_STDDEV_BASELINE = 0.06; // Radians
   * 
   * // Standard deviation multipliers for each camera
   * // (Adjust to trust some cameras more than others)
   * public static final double[] CAMERA_STDDEV_FACTORS =
   * new double[] {
   * 1.0, // left
   * 1.0, // back-left
   * 1.0, // back-right
   * 1.0 // right
   * };
   * double stdDevFactor =
   * Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
   * 
   * double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
   * 
   * double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;
   * 
   * if (cameraIndex < CAMERA_STDDEV_FACTORS.length) {
   * linearStdDev *= CAMERA_STDDEV_FACTORS[cameraIndex];
   * angularStdDev *= CAMERA_STDDEV_FACTORS[cameraIndex];
   * }
   * 
   * // Send vision observation
   * consumer.accept(
   * observation.pose().toPose2d(),
   * observation.timestamp(),
   * VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
   * 
   * 
   */
}
