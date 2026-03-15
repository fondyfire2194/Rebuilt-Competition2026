// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/** Add your docs here. */
public class SimRobotFuelSim {

    private static final DistanceUnit Meters = null;
    private final int CAPACITY = 60;
    private final FuelSim fuelSim;
    private int fuelStored = 8;
    LinearVelocity launchVelocity = MetersPerSecond.of(3);
    CommandSwerveDrivetrain drivetrain;
    HoodSubsystem hood;
    TripleShooterSubsystem shooter;

    double redAngle = 125;
    double blueAngle = 50;

    public SimRobotFuelSim(FuelSim fuelSim, CommandSwerveDrivetrain drivetrain, HoodSubsystem hood,
            TripleShooterSubsystem shooter) {
        this.fuelSim = fuelSim;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
         SmartDashboard.putNumber("FUELTOTAL", fuelStored);
    }

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
        SmartDashboard.putNumber("FUELTOTAL", fuelStored);
    }

    public void launchFuel() {
        if (fuelStored == 0)
            return;
        fuelStored--;
        SmartDashboard.putNumber("FUELTOTAL", fuelStored);
        LinearVelocity initialVelocity = MetersPerSecond.of(6.75);
        double angle = AllianceUtil.isRedAlliance()?redAngle:blueAngle;

        fuelSim.launchFuel(
                initialVelocity,
                Degrees.of(angle),
                // Degrees.of(angle).minus(Degrees.of(HoodSubsystem.getFinalTargetAngle())),
                Degrees.of(drivetrain.getState().Pose.getRotation().getDegrees() + 15),
                LauncherConstants.robotToShooterFuelSim.getMeasureZ());
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters) / 0.54);
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters) * 0.54);
    }

}
