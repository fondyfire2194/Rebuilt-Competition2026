package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.geometry.AllianceFlipUtil;

public class AllianceUtil {

  public static Rotation2d bumpRotation2d = new Rotation2d();

  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red;
    } else {
      return false;
    }
  }

  public static boolean isBlueAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Blue;
    } else {
      return false;
    }
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d flipFieldAngle(Pose2d pose) {
    Translation2d t2d = new Translation2d();
    if (isRedAlliance()) {
      t2d = pose.getTranslation();
      double rads = pose.getRotation().getRadians();
      rads += Math.PI;
      if (rads > Math.PI)
        rads = 2 * Math.PI - rads;
      return new Pose2d(t2d, new Rotation2d(rads));
    } else
      return pose;
  }

  public static Pose2d getHubPose() {
    return isRedAlliance() ? FieldConstants.redHubPose : FieldConstants.blueHubPose;
  }

  public static Pose2d getOutpostPassingPose() {
    return isRedAlliance()
        ? AllianceFlipUtil.apply(FieldConstants.outpostPassingTargetPose)
        : FieldConstants.outpostPassingTargetPose;
  }

  public static Pose2d getDepotPassingPose() {
    return isRedAlliance()
        ? AllianceFlipUtil.apply(FieldConstants.depotPassingTargetPose)
        : FieldConstants.depotPassingTargetPose;
  }

  public static void getBumpCrossAngle(Pose2d robotPose) {

    boolean inNeutralZone = AllianceFlipUtil
        .applyX(robotPose
            .getX()) > FieldConstants.LinesVertical.hubCenter;
    SmartDashboard.putBoolean("AAAAINNEUT", inNeutralZone);
    SmartDashboard.putBoolean("AAAARED", isRedAlliance());
    SmartDashboard.putBoolean("AAAABLUE", isBlueAlliance());

    if (isBlueAlliance())
      if (!inNeutralZone)
        bumpRotation2d = Rotation2d.fromDegrees(-45);
      else
        bumpRotation2d = Rotation2d.fromDegrees(135);

    if (isRedAlliance())
      if (!inNeutralZone)
        bumpRotation2d = Rotation2d.fromDegrees(45);
      else
        bumpRotation2d = Rotation2d.fromDegrees(135);

     }

  public static Pose2d getPassingTargetPose(Pose2d robotPose) {
if(isBlueAlliance())
   return robotPose.getY() < FieldConstants.fieldWidth / 2 ? getOutpostPassingPose() : getDepotPassingPose();
else
   return robotPose.getY() >= FieldConstants.fieldWidth / 2 ? getOutpostPassingPose() : getDepotPassingPose();

  }

  // public static Pose2d getLobPose() {
  // return isRedAlliance() ? FieldConstants.lobRedAlliance :
  // FieldConstants.lobBlueAlliance;
  // }

  // public static Pose2d getSourceShootPose() {
  // return isRedAlliance() ? GeometryUtil
  // .flipFieldPose(FieldConstants.sourceShootBlue) :
  // FieldConstants.sourceShootBlue;
  // }

  // public static Pose2d getSourceClearStagePose() {
  // return isRedAlliance() ? GeometryUtil
  // .flipFieldPose(FieldConstants.sourceClearStagePoseBlue) :
  // FieldConstants.sourceClearStagePoseBlue;
  // }

  // public static Pose2d getAmpClearStagePose() {
  // return isRedAlliance() ? GeometryUtil
  // .flipFieldPose(FieldConstants.ampClearStagePoseBlue) :
  // FieldConstants.ampClearStagePoseBlue;
  // }

}
