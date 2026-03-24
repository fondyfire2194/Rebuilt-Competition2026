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
import frc.robot.Constants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
  // Initialize intake SPARK. We will use open loop control for this.
  private SparkMax intakeMotor = new SparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
  private SparkClosedLoopController intakeClosedLoopController;
  private double intakePowerSim;
  public boolean logData;
  private Timer faultCheckTimer;

  private final Alert intakeAlert = new Alert(
      "Intake Fault",
      AlertType.kError);
  private final Alert intakeCanbusAlert = new Alert(
      "Intake Loss of Canbus",
      AlertType.kError);
  private double faultCheckTime = 5;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(boolean logData) {
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
    intakeMotor.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.logData = logData;
    faultCheckTimer = new Timer();
    faultCheckTimer.start();
    intakeClosedLoopController = intakeMotor.getClosedLoopController();
  }

  public Command clearIntakeStickyFaultsCommand() {
    return Commands.runOnce(() -> intakeMotor.clearFaults());
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void runIntakeMotor(double power) {
    intakeMotor.set(power);
    intakePowerSim = power;
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0);
    intakePowerSim = 0;
  }

  public Command startIntakeCommand() {
    return Commands.runOnce(() -> runIntakeMotor(IntakeSetpoints.kIntake));
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopIntakeMotor());
  }

  public void runIntakeAtVelocity() {
    intakeClosedLoopController.setSetpoint(IntakeSetpoints.kIntakeRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public Command runIntakeAtVelocityCommand() {
    return Commands.run(() -> runIntakeAtVelocity());
  }

  /**
   * Command to run the intake and conveyor motors. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogIntakeCommand() {
    return this.startEnd(
        () -> {
          this.runIntakeMotor(Constants.IntakeSetpoints.kJogIntake);
        }, () -> {
          this.runIntakeMotor(0.0);
        }).withName("Jog Intake");
  }

  /**
   * Command to reverse the intake motor. When the command is
   * interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command jogExtakeCommand() {
    return this.startEnd(
        () -> {
          this.runIntakeMotor(-Constants.IntakeSetpoints.kJogIntake);
        }, () -> {
          this.runIntakeMotor(0.0);
        }).withName("Extaking");
  }

  public double getAppliedOutput() {
    if (RobotBase.isReal())
      return intakeMotor.getAppliedOutput();
    else
      return intakePowerSim;
  }

  public double getIntakeCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  public boolean intakeRunning() {
    if (RobotBase.isReal())
      return Math.abs(getAppliedOutput()) > .1;
    else
      return Math.abs(intakePowerSim) > .1;
  }

  @Override
  public void periodic() {
    if (logData) {
      DogLog.log("Intake/TargetRPM", IntakeSetpoints.kIntakeRPM);
      DogLog.log("Intake/MotorRPM", intakeMotor.getEncoder().getVelocity());
      DogLog.log("Intake/MotorAmps", intakeMotor.getOutputCurrent());
      DogLog.log("Intake/MotorVolts", intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    }
    if (faultCheckTimer.get() > faultCheckTime) {
      intakeAlert.set(intakeMotor.hasActiveFault() || intakeMotor.hasStickyFault());
      intakeCanbusAlert.set(checkIntakeCanFault());
      faultCheckTimer.restart();
    }
  }

  public boolean checkIntakeCanFault() {
    return intakeMotor.getFaults().can;
  }

}
