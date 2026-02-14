// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final class RobotConstants {

    public static double robotOverallLength = Units.inchesToMeters(30);
    public static double robotOverallWidth = Units.inchesToMeters(22);

  }

  public static final class FieldConstants {

    // [images\RebuiltFieldtagLayout.png]

    // [images\AprilTagPositions.png]

    // [\images\FieldDimensions - AM.png]

    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static double fieldLength = aprilTagFieldLayout.getFieldLength();

    public static double fieldWidth = aprilTagFieldLayout.getFieldWidth();

    static double blueStartingLineX = Units.inchesToMeters(156);

    static double blueOutpostSideTrenchStartY = Units.inchesToMeters(24.85);

    static double blueDepotSideTrenchStartY = Units.inchesToMeters(fieldWidth + Units.inchesToMeters(24.85));

    static Pose2d blueOutpostSideTrenchStart = new Pose2d(
        blueStartingLineX - RobotConstants.robotOverallLength / 2,
        blueOutpostSideTrenchStartY,
        new Rotation2d());

    static Pose2d blueDepotSideTrenchStart = new Pose2d(
        blueStartingLineX - RobotConstants.robotOverallLength / 2,
        blueDepotSideTrenchStartY,
        new Rotation2d());

    static Pose2d blueCenterStart = new Pose2d(
        blueStartingLineX - RobotConstants.robotOverallLength / 2,
        Units.inchesToMeters(fieldWidth / 2),
        new Rotation2d());

    public static Pose2d blueHubPose = new Pose2d(
        Units.inchesToMeters(181.56), FieldConstants.fieldWidth / 2, new Rotation2d());

    static double redStartingLineX = Units.inchesToMeters(fieldLength - Units.inchesToMeters(143.5));

    public static Pose2d redHubPose = new Pose2d(
        FieldConstants.fieldLength - Units.inchesToMeters(181.56), FieldConstants.fieldWidth / 2,
        new Rotation2d(Math.PI));

  }

  public static final class IntakeSetpoints {
    public static final double kJogIntake = 0.25;
    public static final double kIntake = 0.6;
    public static final double kExtake = -0.6;
  }

  public static final class FeederSetpoints {
    public static final double kFeedRollerSetpoint = 0.95;
    public static final double kFeedBeltSetpoint = 0.95;

    public static final double kFeedRollerJogSetpoint = 0.5;
    public static final double kFeedBeltJogSetpoint = 0.5;

  }

  public static final class HoodSetpoints {
    public static final double jogHoodMotor = .075;
  }

  public static final class FlywheelSetpoints {
    public static final double kShootRpm = 1000;
    public static final double kVelocityTolerance = 100;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

  }

  public static final class CANIDConstants {

    public static final int intakeID = 10;
    public static final int intakeArmID = 11;

    public static final int feederBeltID = 12;
    public static final int feederRollerID = 13;
    public static final int hoodMotorID = 14;

    // these are CV1 addresses
    public static final int leftShooterID = 10;
    public static final int centerShooterID = 11;
    public static final int rightShooterID = 12;

  }

  public static final class CanbusConstants {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("CV1");

  }

  public static class KrakenX60 {
    public static final AngularVelocity kFreeSpeed = RPM.of(6000);
  }

  public static final class CameraConstants {

    public static class CameraValues {
      public String camname = "name";
      public String ipaddress = "ip";
      public boolean isLL4;
      public Pose3d camPose;
      public double hfov;
      public double vfov;
      public int horpixels;
      public int vertpixels;
      public boolean isUsed = true;
      public int poseUpdateCount = 0;
      public boolean showTelemetry = false;

      public CameraValues(
          final String camname,
          final String ipaddress,
          final boolean isLL4,
          final Pose3d camPose,
          final double hfov, double vfov,
          final int horpixels, final int vertpixels,
          final boolean isUsed) {
        this.camname = camname;
        this.ipaddress = ipaddress;
        this.isLL4 = isLL4;
        this.camPose = camPose;
        this.hfov = hfov;
        this.vfov = vfov;
        this.horpixels = horpixels;
        this.vertpixels = vertpixels;
        this.isUsed = isUsed;

      }
    }

    /**
     * //https://youtu.be/unX1PsPi0VA?si=D1i4hf6OA0_LXidt
     * Pose3d rotation Parameters:
     * Roll is CCW angle around X in radians (normally 0()
     * Pitch is CCW angle around Y in radians (0 is parallel to ground)
     * Yaw is CCW angle around Z axis in radians 90 is pointing left
     * 
     */
    static Pose3d frontCamPose = new Pose3d(
        Units.inchesToMeters(9.5), // front of robot
        Units.inchesToMeters(0), // on LR center
        Units.inchesToMeters(8), // high
        new Rotation3d(
            Units.degreesToRadians(0), // no roll
            Units.degreesToRadians(20), // angled up
            Units.degreesToRadians(0)));// facing forward

    public static CameraValues frontCamera = new CameraValues(
        "limelight-front",
        "10.21.94.15",
        true,
        frontCamPose,
        63.3,
        49.7,
        1,
        1,
        true);

    static Pose3d leftCamPose = new Pose3d(
        Units.inchesToMeters(0),
        Units.inchesToMeters(10),
        Units.inchesToMeters(8),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(20),
            Units.degreesToRadians(100)));

    public static CameraValues leftCamera = new CameraValues(
        "limelight-left",
        "10.21.94.16",
        false,
       leftCamPose,
        63.3,
        49.7,
        1280,
        960,
        true);

    static Pose3d rightCamPose = new Pose3d(
        0.08255,
        0.127,
        0.16256,
        new Rotation3d(
            .08255,
            Units.degreesToRadians(20),
            Units.degreesToRadians(-100)));

    public static CameraValues rightCamera = new CameraValues(
        "limelight-right",
        "10.21.94.17",
        false,
        rightCamPose,
        63.3,
        49.7,
        1280,
        960,
        true);

    public static StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Camposes", Pose3d.struct).publish();

    public static Pose3d[] camPoses = {
        CameraConstants.frontCamera.camPose,
        CameraConstants.leftCamera.camPose,
        CameraConstants.rightCamera.camPose };


    public static int apriltagPipeline = 0;
    public static int viewFinderPipeline = 5;

  }

}
