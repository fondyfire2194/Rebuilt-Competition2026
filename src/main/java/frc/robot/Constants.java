// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

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

    public static double trackWidthY = .5588;

    public static boolean disableHAL = false;

    public static void disableHAL() {
      disableHAL = true;
    }

    public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
    // top // speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per

  }

  public static class Dimensions {
    public static final Distance BUMPER_THICKNESS = Inches.of(3); // frame to edge of bumper
    public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
    public static final Distance FRAME_SIZE_Y = Inches.of(26.25); // left to right (y-axis)
    public static final Distance FRAME_SIZE_X = Inches.of(28.75); // front to back (x-axis)

    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
  }

  public class LauncherConstants {
    public static Transform3d robotToShooter = new Transform3d(0.35, 0.01, 0.599, new Rotation3d(0.0, 0.0, 0));
    public static Transform3d robotToShooterFuelSim = new Transform3d(0., 0., 0.599, new Rotation3d(0.0, 0.0, 0));

    // public static Transform2d robotToLauncher2d = new Transform2d(0.276, 0.0, new
    // Rotation2d(0));
    public static Transform2d toTransform2d(Transform3d transform) {
      return new Transform2d(
          transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
    }

    private LauncherConstants() {
    }
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

    /**
     * Officially defined and relevant horizontal lines found on the field (defined
     * by Y-axis offset)
     *
     * <p>
     * NOTE: The field element start and end are always left to right from the
     * perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

      public static final double center = fieldWidth / 2.0;

      // Right of hub
      public static final double rightBumpStart = Hub.nearRightCorner.getY();
      public static final double rightBumpEnd = rightBumpStart - RightBump.width;
      public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
      public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
      public static final double rightTrenchOpenEnd = 0;

      // Left of hub
      public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
      public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
      public static final double leftBumpMiddle = (leftBumpStart + leftBumpEnd) / 2.0;
      public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
      public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Left Bump related constants */
    public static class LeftBump {

      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
          Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, fieldWidth,
          openingHeight);
      public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter,
          fieldWidth - openingWidth, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter, fieldWidth,
          openingHeight);
      public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter,
          fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, openingWidth,
          openingHeight);
      public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter, openingWidth,
          openingHeight);
      public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter, 0,
          openingHeight);
    }

    /**
     * Officially defined and relevant vertical lines found on the field (defined by
     * X-axis offset)
     */
    /** Hub related constants */
    public static class Hub {

      // Dimensions
      public static final double width = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
      public static final double innerWidth = Units.inchesToMeters(41.7);
      public static final double innerHeight = Units.inchesToMeters(56.5);

      // Relevant reference points on alliance side
      public static final Translation3d topCenterPoint = new Translation3d(
          aprilTagFieldLayout.getTagPose(26).get().getX() + width / 2.0,
          fieldWidth / 2.0,
          height);
      public static final Translation3d innerCenterPoint = new Translation3d(
          aprilTagFieldLayout.getTagPose(26).get().getX() + width / 2.0,
          fieldWidth / 2.0,
          innerHeight);

      public static final Translation2d nearLeftCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
          fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d nearRightCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
          fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d farLeftCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
          fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d farRightCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
          fieldWidth / 2.0 - width / 2.0);

      // Relevant reference points on the opposite side
      public static final Translation3d oppTopCenterPoint = new Translation3d(
          aprilTagFieldLayout.getTagPose(4).get().getX() + width / 2.0,
          fieldWidth / 2.0,
          height);
      public static final Translation2d oppNearLeftCorner = new Translation2d(oppTopCenterPoint.getX() - width / 2.0,
          fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppNearRightCorner = new Translation2d(oppTopCenterPoint.getX() - width / 2.0,
          fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d oppFarLeftCorner = new Translation2d(oppTopCenterPoint.getX() + width / 2.0,
          fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppFarRightCorner = new Translation2d(oppTopCenterPoint.getX() + width / 2.0,
          fieldWidth / 2.0 - width / 2.0);

      // Hub faces
      public static final Pose2d nearFace = aprilTagFieldLayout.getTagPose(26).get().toPose2d();
      public static final Pose2d farFace = aprilTagFieldLayout.getTagPose(20).get().toPose2d();
      public static final Pose2d rightFace = aprilTagFieldLayout.getTagPose(18).get().toPose2d();
      public static final Pose2d leftFace = aprilTagFieldLayout.getTagPose(21).get().toPose2d();
    }

    public static class LinesVertical {
      public static final double center = fieldLength / 2.0;
      public static final double starting = aprilTagFieldLayout.getTagPose(26).get().getX();
      public static final double allianceZone = starting;
      public static final double hubCenter = aprilTagFieldLayout.getTagPose(26).get().getX() + Hub.width / 2.0;
      public static final double neutralZoneNear = center - Units.inchesToMeters(120);
      public static final double neutralZoneFar = center + Units.inchesToMeters(120);
      public static final double oppHubCenter = aprilTagFieldLayout.getTagPose(4).get().getX() + Hub.width / 2.0;
      public static final double oppAllianceZone = aprilTagFieldLayout.getTagPose(10).get().getX();
    }

    public static final double xPassFromFieldEdge = 1;
    public static final double yPassFromFieldEdge = 1;

    public static final Pose2d outpostPassingTargetPose = new Pose2d(xPassFromFieldEdge, yPassFromFieldEdge,
        new Rotation2d());
    public static final Pose2d depotPassingTargetPose = new Pose2d(xPassFromFieldEdge,
        FieldConstants.fieldWidth - yPassFromFieldEdge, new Rotation2d());
  }

  public static final class IntakeSetpoints {
    public static final double kJogIntake = 0.25;
    public static final double kIntake = 0.4;
    public static final double kExtake = -0.4;
  }

  public static final class FeederSetpoints {
    public static final double kFeedRollerSetpoint = 0.95;
    public static final double kFeedBeltSetpoint = 0.95;

    public static final double kFeedRollerJogSetpoint = 0.5;
    public static final double kFeedBeltJogSetpoint = 0.5;

    public static double kRollerShootRPM = 4500;
    public static double rollerSpeedToStartBelt = 3500;

  }

  public static final class HoodSetpoints {
    public static final double jogHoodMotor = .15;
  }

  public static final class FlywheelSetpoints {
    public static final double kShootRpm = 1000;
    public static final double kVelocityTolerance = 100;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

  }

  public static final class CANIDConstants {

    public static final int pdh = 1;

    public static final int intakeID = 10;
    public static final int intakeArmID = 11;
    
    public static final int intakeArmFollowerID = 16;

    public static final int feederBeltID = 13;
    public static final int feederRollerID = 12;

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

    public static class Cameras {
      public String camname = "name";
      public String ipaddress = "ip";
      public boolean isLL4;
      public Pose3d camPose;
      public double hfov;
      public double vfov;
      public int horpixels;
      public int vertpixels;
      public boolean isUsed;

      public Cameras(
          final String camname,
          final String ipaddress,
          final boolean isLL4,
          final Pose3d camPose,
          final double hfov,
          final double vfov,
          final int horpixels,
          final int vertpixels,
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
        Units.inchesToMeters(12), // front of robot
        Units.inchesToMeters(0), // on LR center
        Units.inchesToMeters(19.), // high
        new Rotation3d(
            Units.degreesToRadians(0), // no roll
            Units.degreesToRadians(23), // angled up
            Units.degreesToRadians(0)));// facing forward

    public static Cameras frontCamera = new Cameras(
        "limelight-front",
        "10.21.94.15",
        true,
        frontCamPose,
        63.3,
        49.7,
        1280,
        960,
        true);

    static Pose3d leftCamPose = new Pose3d(
        Units.inchesToMeters(100.),
        Units.inchesToMeters(100),
        Units.inchesToMeters(0.),
        new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(-15),
            Units.degreesToRadians(130)));

    public static Cameras leftCamera = new Cameras(
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

    public static Cameras rightCamera = new Cameras(
        "limelight-right",
        "10.21.94.17",
        false,
        rightCamPose,
        63.3,
        49.7,
        1280,
        960,
        true);

    public static Cameras rearCamera = new Cameras(
        "limelight-rear",
        "10.21.94.18",
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
    public static int fuelDetectorPipeline = 1;
    public static int viewFinderPipeline = 5;

  }

  public static double loopPeriodSecs = .02;

}
