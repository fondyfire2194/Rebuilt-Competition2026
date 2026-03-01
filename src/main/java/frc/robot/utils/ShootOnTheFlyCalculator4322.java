package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

/**
 * Calculator for shooting on the fly (SOTF). Compensates for robot velocity by
 * adjusting robot
 * angle, flywheel RPM, and hood angle.
 *
 * <p>
 * Based on the time-of-flight lookup table approach described at
 * blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
 */
public class ShootOnTheFlyCalculator4322 {

  StructPublisher<Translation2d> robotPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/RobotPose", Translation2d.struct).publish();
  StructPublisher<Translation2d> futurePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/FuturePose", Translation2d.struct).publish();

  StructPublisher<Translation2d> toGoalPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/ToGoal", Translation2d.struct).publish();

  StructPublisher<Translation2d> targetDirectionPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/TargetDirectipn", Translation2d.struct).publish();

  StructPublisher<Translation2d> targetVelocityPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/TargetVelocity", Translation2d.struct).publish();

  StructPublisher<Translation2d> shotVelocityPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/ShotVelocity", Translation2d.struct).publish();

  StructPublisher<Rotation2d> robotAnglePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/RobotAngle", Rotation2d.struct).publish();

  StructPublisher<Pose2d> projectedHubPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("SOTF/ProjectedHubPose", Pose2d.struct).publish();

  /** Parameters stored in the shooter lookup table. */
  public record ShooterParams(double rpm, double hoodAngle, double timeOfFlight) {
  }

  /** Output command from the calculator. */
  public record ShooterCommand(
      Rotation2d robotAngle, double rpm, double hoodAngle, double effectiveDistance) {
  }

  private final InterpolatingTreeMap<Double, ShooterParams> shooterTable;
  public final boolean showData;

  /**
   * Creates a calculator with the provided shooter table.
   *
   * @param shooterTable Interpolating map from distance (meters) to shooter
   *                     parameters
   */
  public ShootOnTheFlyCalculator4322(InterpolatingTreeMap<Double, ShooterParams> shooterTable, boolean showData) {
    this.shooterTable = shooterTable;
    this.showData = showData;
  }

  /**
   * Creates a calculator with a new empty table. Use {@link #addTableEntry} to
   * populate.
   */
  public ShootOnTheFlyCalculator4322(boolean showData) {
    this.showData = showData;
    this.shooterTable = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(),
        (start, end, t) -> new ShooterParams(
            MathUtil.interpolate(start.rpm(), end.rpm(), t),
            MathUtil.interpolate(start.hoodAngle(), end.hoodAngle(), t),
            MathUtil.interpolate(start.timeOfFlight(), end.timeOfFlight(), t)));
  }

  /**
   * Adds an entry to the shooter table.
   *
   * @param distance     Distance in meters
   * @param rpm          Flywheel RPM at this distance
   * @param hoodAngle    Hood angle in degrees at this distance
   * @param timeOfFlight Time of flight in seconds at this distance
   */
  public void addTableEntry(double distance, double rpm, double hoodAngle, double timeOfFlight) {
    shooterTable.put(distance, new ShooterParams(rpm, hoodAngle, timeOfFlight));
  }

  /**
   * Calculates shooter parameters for a stationary robot.
   *
   * @param robotPosition Current robot position on field
   * @param goalPosiiton  Goal/target position on field
   * @return Shooter command with robot angle, RPM, and hood angle
   */
  public ShooterCommand calculateStationary(
      Translation2d robotPosition, Translation2d goalPosition) {
    return calculate(robotPosition, new Translation2d(), goalPosition, 0.0);
  }

  /**
   * Calculates shooter parameters compensating for robot motion.
   *
   * @param robotPosition       Current robot position on field
   * @param robotVelocity       Current robot velocity (field-relative, m/s)
   * @param goalPosition        Goal/target position on field
   * @param latencyCompensation Additional time to project position forward
   *                            (seconds)
   * @return Shooter command with robot angle, RPM, and hood angle
   */
  public ShooterCommand calculate(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {

    // 1. Project future position (account for latency)
    Translation2d futurePosition = robotPosition.plus(robotVelocity.times(latencyCompensation));

    
      robotPublisher.accept(robotPosition);
      futurePublisher.accept(futurePosition);
  
    // 2. Get target vector
    Translation2d toGoal = goalPosition.minus(futurePosition);
    toGoalPublisher.accept(toGoal);
    double distance = toGoal.getNorm();

    SmartDashboard.putNumber("SOTF/DistToGoal", distance);

    Translation2d targetDirection = toGoal.div(distance);

    targetDirectionPublisher.accept(targetDirection);

    // 3. Look up baseline parameters from table
    ShooterParams baseline = shooterTable.get(distance);

    double baselineVelocity = distance / baseline.timeOfFlight();

    SmartDashboard.putNumber("SOTF/BaselineVel", baselineVelocity);

    // 4. Build target velocity vector (velocity needed to reach goal)
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);
    targetVelocityPublisher.accept(targetVelocity);
    // 5. THE MAGIC: subtract robot velocity to get required shot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    shotVelocityPublisher.accept(shotVelocity);
    // 6. Extract robot angle and required velocity magnitude
    Rotation2d robotAngle = shotVelocity.getAngle();
    robotAnglePublisher.accept(robotAngle);
    double requiredVelocity = shotVelocity.getNorm();
    SmartDashboard.putNumber("SOTF/RequireVel", requiredVelocity);

    // 7. Use Option 3: adjust both RPM and hood angle
    ShooterCommand adjusted = calculateBothAdjustments(distance, baseline, requiredVelocity);
    Pose2d projectedOnTheMoveShootPose = getProjectedHubPose(futurePosition,
            distance, robotAngle);
projectedHubPosePublisher.accept(projectedOnTheMoveShootPose);
       SmartDashboard.putNumber("SOTF/projectedYDiff",
            projectedOnTheMoveShootPose.getY() - FieldConstants.redHubPose.getY());
    return new ShooterCommand(
        robotAngle, adjusted.rpm(), adjusted.hoodAngle(), adjusted.effectiveDistance());
  }

  /**
   * Option 3: Adjusts both RPM and hood angle to achieve required velocity.
   * Splits the correction
   * evenly between the two systems using sqrt for equal contribution.
   *
   * @param distance         Actual distance to target
   * @param baseline         Baseline parameters from lookup table
   * @param requiredVelocity Required horizontal velocity magnitude
   * @return Adjusted shooter command
   */
  private ShooterCommand calculateBothAdjustments(
      double distance, ShooterParams baseline, double requiredVelocity) {

    /*
     * baseline is how fast you need to shoot given 0 velocity robot movement
     * requiredVelocity is how fast the ball needs to exit your shooter when
     * accounting for robot velocity
     * 
     * Tof(x) is map from distance -> time traveled.
     * Vel(distance) = distance / Tof(distance) = velocity
     * 
     * we'll use newton's method to figure out wwhat distance input generates our
     * desired velocity.
     * we can look for the roots of f(x) = Vel(x) - requiredVelocity to get us our
     * shooter params,
     * where Vel(x) = x/Tof(x)
     * 
     * x_n+1 = x_n - f(x_n) / f'(x_n)
     * 
     * f(x_n) = currentVelocity - requiredVelocity
     * f'(x_n) = Vel'(x)
     * 
     */

    ShooterParams currentParams = baseline;
    double currentDistance = distance;
    double currentTime = currentParams.timeOfFlight();
    double currentVelocity = currentDistance / currentTime;

    // 10 rounds for now
    for (int i = 0; i < 10 && Math.abs(currentVelocity - requiredVelocity) > 0.005; i++) {
      // estimate d/dx ((Vel(x)) - requiredVelocity) = d/dx (Vel(x)) by taking a tiny
      // slope
      final double EPSILON = 0.001;
      double lowVel = (currentDistance - EPSILON) / shooterTable.get(currentDistance - EPSILON).timeOfFlight();
      double highVel = (currentDistance + EPSILON) / shooterTable.get(currentDistance + EPSILON).timeOfFlight();
      double velDeriv = (highVel - lowVel) / (EPSILON * 2);
      currentDistance -= (currentVelocity - requiredVelocity) / velDeriv;
      // update currentVelocity with f(x+1)
      currentParams = shooterTable.get(currentDistance);
      currentTime = currentParams.timeOfFlight();
      currentVelocity = currentDistance / currentTime;
    }
    return new ShooterCommand(
        null, currentParams.rpm(), currentParams.hoodAngle(), currentDistance);
  }

  
  public Pose2d getProjectedHubPose(Translation2d robotPose, double distanceToHub, Rotation2d driveAngle) {

    /**
     * calculate the pose where the computed drive angle hits the x center line of
     * the hub
     * Use the distance to the hub as the hypotenuse and break it into x and y
     * components
     * 
     */

    double xDistToHub = distanceToHub * driveAngle.getCos();
    double yDistToHub = distanceToHub * driveAngle.getSin();
    Pose2d temp = new Pose2d(robotPose.getX() + xDistToHub, robotPose.getY() + yDistToHub, driveAngle);

    return temp;

  }

}
