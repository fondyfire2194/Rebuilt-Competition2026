// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.CanbusConstants;

public class TripleShooterSubsystem extends SubsystemBase {
  /** Creates a new TripleShooterSubsystem. */
  private static final AngularVelocity kVelocityTolerance = RPM.of(100);

  public final TalonFX leftMotor;

  public final TalonFX middleMotor;

  private final TalonFX rightMotor;

  public boolean leftMotorActive;
  public boolean middleMotorActive;
  public boolean rightMotorActive;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  StatusCode statusL = StatusCode.StatusCodeNotInitialized;
  StatusCode statusM = StatusCode.StatusCodeNotInitialized;
  StatusCode statusR = StatusCode.StatusCodeNotInitialized;

  private DutyCycleOut dc_out = new DutyCycleOut(0.0);

  private VoltageOut volts_out = new VoltageOut(0);

  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private Distance distanceToHub;

  // private AngularVelocity targetVelocity = RPM.of(500);
  private double targetRPM = 500;
  private AngularAcceleration targetAcceleration = RotationsPerSecondPerSecond.of(50);

  private int tst = 0;

  public TripleShooterSubsystem(boolean showData) {

    leftMotor = new TalonFX(CANIDConstants.leftShooterID, CanbusConstants.kCANivoreCANBus);
    middleMotor = new TalonFX(CANIDConstants.centerShooterID, CanbusConstants.kCANivoreCANBus);
    rightMotor = new TalonFX(CANIDConstants.rightShooterID, CanbusConstants.kCANivoreCANBus);

    configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
    configureMotor(middleMotor, InvertedValue.Clockwise_Positive);
    configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

    if (showData)
      SmartDashboard.putData(this);
  }

  private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    final TalonFXConfiguration configs = new TalonFXConfiguration();

    /*
     * Voltage-based velocity requires a velocity feed forward to account for the
     * back-emf of the motor
     */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / rotation per second
    configs.Slot0.kP = 0.2; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

    /*
     * Torque-based velocity does not require a velocity feed forward, as torque
     * will accelerate the rotor up to the desired velocity by itself
     */
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));

    /* Retry config apply up to 5 times, report if failure */

    for (int i = 0; i < 5; ++i) {
      statusL = leftMotor.getConfigurator().apply(configs);
      if (statusL.isOK())
        break;
    }
    for (int i = 0; i < 5; ++i) {
      statusM = middleMotor.getConfigurator().apply(configs);
      if (statusM.isOK())
        break;
    }
    for (int i = 0; i < 5; ++i) {
      statusR = rightMotor.getConfigurator().apply(configs);
      if (statusR.isOK())
        break;
    }
    if (!statusL.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusL.toString());
    }
    if (!statusM.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusM.toString());
    }
    if (!statusR.isOK()) {
      System.out.println("Could not apply configs, error code: " + statusR.toString());
    }

  }

  public void runVelocityVoltage(TalonFX motor) {
    motor.setControl(
        velocityVoltage
            .withVelocity(RPM.of(targetRPM))
            .withAcceleration(targetAcceleration)
            .withEnableFOC(true));
  }

  public Command runVelocityVoltageCommand(TalonFX motor) {
    return run(() -> runVelocityVoltage(motor));
  }

  public Command runAllVelocityVoltageCommand() {
    return run(() -> runAllVelocityVoltage());
  }

  public void runAllVelocityVoltage() {
    if (leftMotorActive)
      runVelocityVoltage(leftMotor);

    if (middleMotorActive)
       runVelocityVoltage(middleMotor);

    if (rightMotorActive)
       runVelocityVoltage(rightMotor);
  }

  public void runVelocityTorque(TalonFX motor) {
    motor.setControl(
        velocityTorque
            .withVelocity(targetRPM));
  }

  public void setPercentOutput(TalonFX motor, double percentOutput) {
    motor.setControl(
        voltageRequest
            .withOutput(Volts.of(percentOutput * 12.0)));
  }

  public void stopAllShooters() {
    leftMotor.stopMotor();
    setPercentOutput(middleMotor, 0.0);
    setPercentOutput(rightMotor, 0.0);
  }

  public Command stopAllShootersCommand() {
    return runOnce(this::stopAllShooters);
  }

  public void disableShooter(TalonFX motor) {
    motor.setControl(m_brake);
  }

  public void disableAllShooters() {
    leftMotor.setControl(m_brake);
    middleMotor.setControl(m_brake);
    rightMotor.setControl(m_brake);
  }

  public Command disableAllShootersCommand() {
    return runOnce(this::disableAllShooters);
  }

  public void setTargetVelocity(double RPM) {
    targetRPM = RPM;
  }

  public Command setTargetVelocityCommand(AngularVelocity vel) {
    return Commands.runOnce(() -> setTargetVelocity(vel.in(RPM)));
  }

  public Command changeTargetVelocityCommand(double rpm) {
    return Commands.runOnce(() -> changeTargetVelocity(rpm));
  }

  public void changeTargetVelocity(double rpm) {
    targetRPM += rpm;
    SmartDashboard.putNumber("TGTrpm", targetRPM);
  }

  public void setDutyCycleOut(TalonFX motor, double dutyCycle) {
    motor.setControl(dc_out.withOutput(dutyCycle));
  }

  public Command setDutyCycleCommand(TalonFX motor, double dutyCycle) {
    return run((() -> setDutyCycleOut(motor, dutyCycle)));
  }

  public void setVoltageOut(TalonFX motor, double volts) {
    motor.setControl(volts_out.withOutput(volts));
  }

  public Command setVoltageCommand(TalonFX motor, double volts) {
    return run((() -> setVoltageOut(motor, volts)));
  }

  public boolean isVelocityWithinTolerance(TalonFX motor) {
    final boolean isInVelocityVoltageMode = motor.getAppliedControl().equals(velocityVoltage);
    final AngularVelocity currentVelocity = motor.getVelocity().getValue();
    final AngularVelocity targetVelocity = velocityVoltage.getVelocityMeasure();
    return isInVelocityVoltageMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
  }

  public boolean allVelocityInTolerance() {
    return (isVelocityWithinTolerance(leftMotor) || !leftMotorActive)
        && (isVelocityWithinTolerance(middleMotor) || !middleMotorActive)
        && (isVelocityWithinTolerance(rightMotor) || !rightMotorActive);
  }

  private void initSendable(SendableBuilder builder, TalonFX motor, String name) {
    builder.addDoubleProperty(name + " RPM", () -> motor.getVelocity().getValue().in(RPM), null);
    builder.addDoubleProperty(name + " Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(name + " Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    builder.addDoubleProperty(name + " Supply Volts", () -> motor.getMotorVoltage().getValueAsDouble(), null);
    builder.addBooleanProperty(name + " At Speed", () -> isVelocityWithinTolerance(motor), null);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // if (leftMotorActive)
    initSendable(builder, leftMotor, "Left");
    // if (middleMotorActive)
    initSendable(builder, middleMotor, "Middle");
    // if (rightMotorActive)
    // initSendable(builder, rightMotor, "Right");
    builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addDoubleProperty("Voltage Target RPM", () -> velocityVoltage.getVelocityMeasure().in(RPM), null);
    builder.addDoubleProperty("Target Velocity RPM", () -> targetRPM, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SHTRTST", tst);
  }

  public void setDistanceToHub(double distance) {
    distanceToHub = Meters.of(distance);
  }

  public Distance getDistanceToHub() {
    return distanceToHub;
  }
}
