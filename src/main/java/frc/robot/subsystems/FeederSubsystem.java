// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FeederSetpoints;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  public SparkMax feederBeltMotor;

  public SparkMax feederRollerMotor;

  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double feederRollerPowerSim;
  private double feederBeltPowerSim;

  private boolean showData;

  private final Alert feederAlert = new Alert(
      "Feeder Fault",
      AlertType.kError);

  public boolean pulse;

  public double beltStartPulseTime = 2.;
  public double beltPulseTime = .5;

  public double beltStopPulseTime = beltStartPulseTime + beltPulseTime;

  public double beltInitialShootTime = 5.;

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

    closedLoopController = feederRollerMotor.getClosedLoopController();

    feederBeltMotor.configure(
        Configs.Feeder.feederBeltConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.showData = showData;
    if (showData)
      SmartDashboard.putData(this);

    feederAlert.set(feederBeltMotor.hasActiveFault() || feederBeltMotor.hasStickyFault()
        || feederRollerMotor.hasActiveFault() || feederRollerMotor.hasStickyFault());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Feeder");
    builder.addDoubleProperty("Belt Motor RPM", () -> feederBeltMotor.getEncoder().getVelocity(), null);
    builder.addDoubleProperty("Belt Motor Amps", () -> feederBeltMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Belt Motor Sim", () -> feederBeltPowerSim, null);
    builder.addBooleanProperty("Belt Motor Fault", () -> feederBeltMotor.hasActiveFault(), null);

    builder.addDoubleProperty("Roller Motor RPM", () -> feederRollerMotor.getEncoder().getVelocity(), null);
    builder.addDoubleProperty("Roller Motor Amps", () -> feederRollerMotor.getOutputCurrent(), null);
    builder.addBooleanProperty("Roller Motor Fault", () -> feederRollerMotor.hasActiveFault(), null);

  }

  @Override
  public void periodic() {
    

      // This method will be called once per scheduler run
      DogLog.log("Feeder/RollerRPM", feederRollerMotor.getEncoder().getVelocity());
      DogLog.log("Feeder/RollerTargetRPM", closedLoopController.getSetpoint());
      DogLog.log("Feeder/RollerAmps", feederRollerMotor.getOutputCurrent());
      DogLog.log("Feeder/RollerVolts", feederRollerMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

      DogLog.log("Feeder/BeltRPM", feederBeltMotor.getEncoder().getVelocity());
      DogLog.log("Feeder/BeltAmps", feederBeltMotor.getOutputCurrent());
      DogLog.log("Feeder/BeltVolts", feederBeltMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    

  }

  public void runFeederRollerMotor(double power) {
    feederRollerMotor.set(power);
    feederRollerPowerSim = power;
  }

  public void runFeederRollerAtVelocity() {
    closedLoopController.setSetpoint(FeederSetpoints.kRollerShootRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void stopFeederRollerMotor() {
    closedLoopController.setSetpoint(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    feederRollerMotor.set(0);
    feederRollerPowerSim = 0;
  }

  public Command startFeederRollerCommand() {
    return Commands.runOnce(() -> runFeederRollerMotor(FeederSetpoints.kFeedRollerSetpoint));
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
          this.runFeederRollerMotor(Constants.FeederSetpoints.kFeedRollerJogSetpoint);
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
          this.runFeederRollerMotor(-Constants.FeederSetpoints.kFeedRollerJogSetpoint);
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

  // feeder belt

  public void runFeederBeltMotor(double power) {
    feederBeltMotor.set(power);
    feederBeltPowerSim = power;
  }

  public void stopFeederBeltMotor() {
    feederBeltMotor.set(0);
    feederBeltPowerSim = 0;
  }

  public Command startFeederBeltCommand() {
    return Commands.runOnce(() -> runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint));
  }

  public Command stopFeederBeltCommand() {
    return Commands.runOnce(() -> stopFeederBeltMotor());
  }

  public void runBeltAandRollers() {
    runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint);
    runFeederRollerMotor(FeederSetpoints.kFeedRollerSetpoint);
  }

  public Command runBeltsAndRollersCommand() {
    return Commands.runOnce(() -> runBeltAandRollers());
  }

  /**
   * Command to run the feeder belt motor. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogFeederBeltCommand() {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(Constants.FeederSetpoints.kFeedBeltJogSetpoint);
        }, () -> {
          this.runFeederBeltMotor(0.0);
        }).withName("JogFeederBelt");
  }

  /**
   * Command to reverse the feeder belt motor. When the command is
   * interrupted, e.g. the button is released, the motors will stop.
   */
  public Command jogReverseFeederBeltCommand() {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(-Constants.FeederSetpoints.kFeedBeltSetpoint);
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

  public Command clearFeederStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> feederRollerMotor.clearFaults()),
        Commands.runOnce(() -> feederBeltMotor.clearFaults()));
  }

  public void pulseBelt() {
    runFeederBeltMotor(-.25);
  }

}
