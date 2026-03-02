// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;

/** Add your docs here. */
public class ShootingData {

    // Launching Maps
    public static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    // Passing Maps
    public static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap passingShooterSpeedMap = new InterpolatingDoubleTreeMap();
    public final static InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    // Passing targets
    public static final double hubPassLine = FieldConstants.LinesHorizontal.rightBumpStart
            - RobotConstants.trackWidthY / 2.0;
    public static final double xPassTarget = Units.inchesToMeters(25);
    public static final double yPassTarget = Units.inchesToMeters(50);

    
  public static final double minDistance = .9;
  public static final double maxDistance = 4.9;
  public static final double passingMinDistance = 0;
  public static final double passingMaxDistance = 12;

    static {

        hoodAngleMap.put(0.96, Rotation2d.fromDegrees(10.0));
        hoodAngleMap.put(1.16, Rotation2d.fromDegrees(12.0));
        hoodAngleMap.put(1.58, Rotation2d.fromDegrees(14.0));
        hoodAngleMap.put(2.07, Rotation2d.fromDegrees(18.5));
        hoodAngleMap.put(2.37, Rotation2d.fromDegrees(22.0));
        hoodAngleMap.put(2.47, Rotation2d.fromDegrees(23.0));
        hoodAngleMap.put(2.70, Rotation2d.fromDegrees(24.0));
        hoodAngleMap.put(2.94, Rotation2d.fromDegrees(25.0));
        hoodAngleMap.put(3.48, Rotation2d.fromDegrees(27.0));
        hoodAngleMap.put(3.92, Rotation2d.fromDegrees(32.0));
        hoodAngleMap.put(4.35, Rotation2d.fromDegrees(34.0));
        hoodAngleMap.put(4.84, Rotation2d.fromDegrees(38.0));

        shooterSpeedMap.put(0.96, 1500.0);
        shooterSpeedMap.put(1.16, 1550.0);
        shooterSpeedMap.put(1.58, 1600.0);
        shooterSpeedMap.put(2.07, 1650.0);
        shooterSpeedMap.put(2.37, 1700.0);
        shooterSpeedMap.put(2.47, 1700.0);
        shooterSpeedMap.put(2.70, 1700.0);
        shooterSpeedMap.put(2.94, 1750.0);
        shooterSpeedMap.put(3.48, 1750.0);
        shooterSpeedMap.put(3.92, 1800.0);
        shooterSpeedMap.put(4.35, 1850.0);
        shooterSpeedMap.put(4.84, 1900.0);

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);

        passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(0.0));
        passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(3.0));
        passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(8.0));

        passingShooterSpeedMap.put(5.46, 160.0);
        passingShooterSpeedMap.put(6.62, 180.0);
        passingShooterSpeedMap.put(7.80, 200.0);

        passingTimeOfFlightMap.put(passingMinDistance, 0.0);
        passingTimeOfFlightMap.put(passingMaxDistance, 0.0);

    }

    public static double getMinTimeOfFlight() {
        return timeOfFlightMap.get(minDistance);
    }

    public static double getMaxTimeOfFlight() {
        return timeOfFlightMap.get(maxDistance);
    }

}
