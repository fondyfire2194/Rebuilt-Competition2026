// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FeederSetpoints;
import frc.robot.Constants.IntakeSetpoints;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private SparkMax feederBeltMotor;
  private SparkMax feederRollerMotor;
  
  private double feederRollerPowerSim;
  private double feederBeltPowerSim;

  public FeederSubsystem(boolean showData) {
    feederBeltMotor = new SparkMax(Constants.CANIDConstants.feederBeltID, MotorType.kBrushless);
    feederRollerMotor = new SparkMax(Constants.CANIDConstants.feederRollerID, MotorType.kBrushless);
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    feederRollerMotor.configure(
        Configs.Feeder.feederRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    feederBeltMotor.configure(
        Configs.Feeder.feederBeltConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        
    if (showData)
      SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void runFeederRollerMotor(double power) {
    feederRollerMotor.set(power);
    feederRollerPowerSim = power;
  }

  private void stopFeederRollerMotor() {
    feederRollerMotor.set(0);
    feederRollerPowerSim = 0;
  }

  public Command startFeederRollerCommand() {
    return Commands.runOnce(() -> runFeederRollerMotor(IntakeSetpoints.kIntake));
  }

  public Command stopFeederRollerCommand() {
    return Commands.runOnce(() -> stopFeederRollerMotor());
  }

  /**
   * Command to run the feeder roller motor. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogFeederRollerCommand() {
    return this.startEnd(
        () -> {
          this.runFeederRollerMotor(Constants.IntakeSetpoints.kIntake);
        }, () -> {
          this.runFeederRollerMotor(0.0);
        }).withName("Jog FeederRoller");
  }

  /**
   * Command to reverse the feeder roller motor. When the command is
   * interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command jogReverseFeederRollerCommand() {
    return this.startEnd(
        () -> {
          this.runFeederRollerMotor(Constants.FeederSetpoints.kFeedRollerSetpoint);
        }, () -> {
          this.runFeederRollerMotor(0.0);
        }).withName("FeederRollerReversing");
  }

  public double getFeederRollerAppliedOutput() {
    if (RobotBase.isReal())
      return feederRollerMotor.getAppliedOutput();
    else
      return feederRollerPowerSim;
  }

  public double getFeederRollerCurrent() {
    return feederRollerMotor.getOutputCurrent();
  }

  public boolean feederRollerRunning() {
    return Math.abs(getFeederRollerAppliedOutput()) > .1;
  }

//feeder belt

  
  private void runFeederBeltMotor(double power) {
    feederBeltMotor.set(power);
    feederBeltPowerSim = power;
  }

  private void stopFeederBeltMotor() {
    feederBeltMotor.set(0);
    feederBeltPowerSim = 0;
  }

  public Command startFeederBeltCommand() {
    return Commands.runOnce(() -> runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint));
  }

  public Command stopFeederBeltCommand() {
    return Commands.runOnce(() -> stopFeederBeltMotor());
  }

  /**
   * Command to run the feeder belt motor. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogFeederBeltCommand() {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(Constants.FeederSetpoints.kFeedBeltSetpoint);
        }, () -> {
          this.runFeederBeltMotor(0.0);
        }).withName("JogFeederBelt");
  }

  /**
   * Command to reverse the feeder belt motor. When the command is
   * interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command jogReverseFeederBeltCommand() {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(Constants.FeederSetpoints.kFeedBeltSetpoint);
        }, () -> {
          this.runFeederBeltMotor(0.0);
        }).withName("FeederBeltReversing");
  }

  public double getFeederBeltAppliedOutput() {
    if (RobotBase.isReal())
      return feederBeltMotor.getAppliedOutput();
    else
      return feederBeltPowerSim;
  }

  public double getFeederBeltCurrent() {
    return feederBeltMotor.getOutputCurrent();
  }

  public boolean feederBeltRunning() {
    return Math.abs(getFeederBeltAppliedOutput()) > .1;
  }


}
