// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/** Add your docs here. */
public class SimRobotFuelSim {

    private final int CAPACITY = 30;
    private final FuelSim fuelSim;
    private int fuelStored = 800;
    LinearVelocity launchVelocity = MetersPerSecond.of(3);
    CommandSwerveDrivetrain drivetrain;
    HoodSubsystem hood;
    TripleShooterSubsystem shooter;
    public double angle = 0;

    public SimRobotFuelSim(FuelSim fuelSim, CommandSwerveDrivetrain drivetrain, HoodSubsystem hood,
            TripleShooterSubsystem shooter) {
        this.fuelSim = fuelSim;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
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
        SmartDashboard.putNumber("AAAAAAAAAFUEL", fuelStored);
        SmartDashboard.putNumber("AAAAAAAAAngle", shooter.shooterLinearVelocity.in(MetersPerSecond));
        SmartDashboard.putNumber("AAAAAAAARPM", shooter.finalSetTargetRPM);
        
      SmartDashboard.putNumber("AAAAAAAAAAAAAAAVel", angle);

        fuelSim.launchFuel(
                shooter.shooterLinearVelocity,
                Degrees.of(angle),
                // Degrees.of(angle).minus(Degrees.of(HoodSubsystem.getFinalTargetAngle())),
                Degrees.of(drivetrain.getState().Pose.getRotation().getDegrees()),
                LauncherConstants.robotToShooterFuelSim.getMeasureZ());
    }

    public Command incAngle() {
        return Commands.runOnce(() -> angle += 10);
    }

}
