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

        hoodAngleMap.put(1.73, Rotation2d.fromDegrees(4.));
        hoodAngleMap.put(2.3, Rotation2d.fromDegrees(6.));
        hoodAngleMap.put(3., Rotation2d.fromDegrees(7.5));
        hoodAngleMap.put(4.33, Rotation2d.fromDegrees(9));
        hoodAngleMap.put(5.04, Rotation2d.fromDegrees(9.0));
        hoodAngleMap.put(6., Rotation2d.fromDegrees(9.5));

        // hoodAngleMap.put(2.70, Rotation2d.fromDegrees(9));
        // hoodAngleMap.put(2.94, Rotation2d.fromDegrees(10.5));
        // hoodAngleMap.put(3.48, Rotation2d.fromDegrees(12));
        // hoodAngleMap.put(3.92, Rotation2d.fromDegrees(13.5));
        // hoodAngleMap.put(4.35, Rotation2d.fromDegrees(14.0));
        // hoodAngleMap.put(4.84, Rotation2d.fromDegrees(15.0));

        shooterSpeedMap.put(1.73, 2600.0 * 1.05);
        shooterSpeedMap.put(2.3, 2800.0 * 1.05);
        shooterSpeedMap.put(3., 3000.0 * 1.05);
        shooterSpeedMap.put(4.33, 3500.0 * 1.05);
        shooterSpeedMap.put(5.04, 3800.0 * 1.05);
        shooterSpeedMap.put(6., 4100. * 1.05);

        // shooterSpeedMap.put(2.70, 3500.0);
        // shooterSpeedMap.put(2.94, 3850.0);
        // shooterSpeedMap.put(3.48, 4200.0);
        // shooterSpeedMap.put(3.92, 4550.0);
        // shooterSpeedMap.put(4.35, 4850.0);
        // shooterSpeedMap.put(4.84, 5000.0);

        timeOfFlightMap.put(5.68, .7);
        timeOfFlightMap.put(4.55, .77);
        timeOfFlightMap.put(3.15, .8);
        timeOfFlightMap.put(1.88, .88);
        timeOfFlightMap.put(1.38, 0.90);

        passingHoodAngleMap.put(5.0, Rotation2d.fromDegrees(8.0));
        passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(8.0));
        passingHoodAngleMap.put(8.0, Rotation2d.fromDegrees(8.0));

        passingShooterSpeedMap.put(5.00, 3500.);
        passingShooterSpeedMap.put(6.62, 3800.0);
        passingShooterSpeedMap.put(8.0, 3900.0);

        passingTimeOfFlightMap.put(passingMinDistance, 1.);
        passingTimeOfFlightMap.put(passingMaxDistance, 1.5);

    }

    public static double getMinTimeOfFlight() {
        return timeOfFlightMap.get(minDistance);
    }

    public static double getMaxTimeOfFlight() {
        return timeOfFlightMap.get(maxDistance);
    }

}
