// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FeederSetpoints;

public class FeederSubsystem extends SubsystemBase {
  /**
   * Creates a new FeederSubsystem.
   * Fuel travels from belt to belt roller to roller rollers to shooter roller
   * 
   * Feeder belt has a 3:1 reduction from a Neo motor and has 2 inch dia rollers
   * Feeder roller is 1:1 from a Neo motor and has 2 in dia rollers
   * Shooter is 1:1 from Kraken X60 and has a 4" diameter roller
   * 
   * Shooter speed typical is around 3000 rpm - 50 revs per second so 50 * PI * 4
   * or 200 PI inches per second
   * For rollers to provide 50% of that, 50 * PI * 2, rollers need to run same
   * speed as shooter or 3000 rpm
   * For belt to provide 50% of rollers, need 25 * PI * 2 or 1500 rpm but with 3:1
   * reduction means 4500 motor rpm
   * Belt 3600 rpm gives belt roller 1200 rpm so 20 * PI * 2 or 40 * PI inches per
   * second or 20% shooter speed
   * Belt moves 4 inches per roller rev = 20 * 4 or 80 inches per second
   * 
   * Speeds = 80(belt) to 120 to 314 to 628 inches per second
   * 
   * 
   */

  public SparkMax feederBeltMotor;
  public SparkClosedLoopController beltClosedLoopController;

  public SparkMax feederRollerMotor;

  private SparkClosedLoopController rollerClosedLoopController;

  private double feederRollerPowerSim;
  private double feederBeltPowerSim;

  private boolean logData;

  private final Alert feederBeltAlert = new Alert(
      "Feeder Belt Fault",
      AlertType.kError);
  private final Alert feederRollerAlert = new Alert(
      "Feeder Roller Fault",
      AlertType.kError);
  private final Alert feederBeltCanbusAlert = new Alert(
      "Feeder Belt Loss of Canbus",
      AlertType.kError);
  private final Alert feederRollerCanbusAlert = new Alert(
      "Feeder Roller Loss of Canbus",
      AlertType.kError);

  private Timer faultCheckTimer;
  private double faultCheckTime = 5.1;

 
  private boolean alternate;

  public FeederSubsystem(boolean logData) {
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

    rollerClosedLoopController = feederRollerMotor.getClosedLoopController();

    feederBeltMotor.configure(
        Configs.Feeder.feederBeltConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    beltClosedLoopController = feederBeltMotor.getClosedLoopController();

    this.logData = logData;

    faultCheckTimer = new Timer();
    faultCheckTimer.start();

  }

  @Override
  public void periodic() {

    if (faultCheckTimer.get() > faultCheckTime) {
      feederBeltAlert.set(feederBeltMotor.hasActiveFault() || feederBeltMotor.hasStickyFault());
      feederRollerAlert.set(feederRollerMotor.hasActiveFault() || feederRollerMotor.hasStickyFault());

      feederBeltCanbusAlert.set(checkFeederBeltCanFault());
      feederRollerCanbusAlert.set(checkFeederRollerCanFault());
      faultCheckTimer.restart();

    } else {
      if (logData) {

        if (alternate) {
          DogLog.log("Feeder/RollerRPM", feederRollerMotor.getEncoder().getVelocity());
          DogLog.log("Feeder/RollerTargetRPM", rollerClosedLoopController.getSetpoint());
          DogLog.log("Feeder/RollerAmps", feederRollerMotor.getOutputCurrent());
          DogLog.log("Feeder/RollerVolts", feederRollerMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        } else {
          DogLog.log("Feeder/BeltTargetRPM", beltClosedLoopController.getSetpoint());
          DogLog.log("Feeder/BeltRPM", feederBeltMotor.getEncoder().getVelocity());
          DogLog.log("Feeder/BeltAmps", feederBeltMotor.getOutputCurrent());
          DogLog.log("Feeder/BeltVolts", feederBeltMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        }
        alternate = !alternate;
      }
    }
  }

  public boolean checkFeederBeltCanFault() {
    return feederBeltMotor.getFaults().can;
  }

  public boolean checkFeederRollerCanFault() {
    return feederRollerMotor.getFaults().can;
  }

  public void runFeederRollerMotor(double power) {
    feederRollerMotor.set(power);
    feederRollerPowerSim = power;
  }

  public void runFeederRollerAtVelocity() {
    rollerClosedLoopController.setSetpoint(FeederSetpoints.kRollerShootRPM, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0);
  }

  public void stopFeederRollerMotor() {
    rollerClosedLoopController.setSetpoint(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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

  public void runFeederBeltAtVelocity() {
    beltClosedLoopController.setSetpoint(FeederSetpoints.kBeltShootRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void runFeederBeltAtVelocity(double rpm) {
    beltClosedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
