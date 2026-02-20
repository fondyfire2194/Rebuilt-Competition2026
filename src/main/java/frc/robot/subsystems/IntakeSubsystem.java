// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
  // Initialize intake SPARK. We will use open loop control for this.
  private SparkMax intakeMotor = new SparkMax(Constants.CANIDConstants.intakeID, MotorType.kBrushless);
  private double intakePowerSim;
  public boolean showData;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(boolean showData) {
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
        this.showData = showData;
    if (showData)
      SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    builder.addDoubleProperty("Amps", () -> intakeMotor.getOutputCurrent(), null);
    builder.addBooleanProperty("Fault", () -> intakeMotor.hasActiveFault(), null);
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void runIntakeMotor(double power) {
    intakeMotor.set(power);
    intakePowerSim = power;
  }

  private void stopIntakeMotor() {
    intakeMotor.set(0);
    intakePowerSim = 0;
  }

  public Command startIntakeCommand() {
    return Commands.runOnce(() -> runIntakeMotor(IntakeSetpoints.kIntake));
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopIntakeMotor());
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
    return Math.abs(getAppliedOutput()) > .1;
  }

  @Override
  public void periodic() {

  }

}
