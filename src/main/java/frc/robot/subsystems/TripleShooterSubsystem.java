// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.CanbusConstants;
import frc.robot.utils.Logger;

public class TripleShooterSubsystem extends SubsystemBase {
  /** Creates a new TripleShooterSubsystem. */
  private static final AngularVelocity kVelocityTolerance = RPM.of(100);

  public LinearVelocity shooterLinearVelocity = MetersPerSecond.of(1);

  private final Distance shooterRollerDiameter = Inches.of(3.);

  public final TalonFX leftMotor;

  public final TalonFX middleMotor;

  public final TalonFX rightMotor;

  public boolean leftMotorActive;
  public boolean middleMotorActive;
  public boolean rightMotorActive;

  private boolean showData;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public static StatusCode statusL = StatusCode.StatusCodeNotInitialized;
  public static StatusCode statusM = StatusCode.StatusCodeNotInitialized;
  public static StatusCode statusR = StatusCode.StatusCodeNotInitialized;

  private DutyCycleOut dc_out = new DutyCycleOut(0.0);

  private VoltageOut volts_out = new VoltageOut(0);

  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private Distance distanceToHub;

  private double manualSetTargetRPM = 2000;

  public double autoSetTargetRPM = 2500;

  public void setAutoSetTargetRPM(double autoSetTargetRPM) {
    this.autoSetTargetRPM = autoSetTargetRPM;
    finalSetTargetRPM = autoSetTargetRPM;
    shooterLinearVelocity = angularToLinearVelocity(RPM.of(finalSetTargetRPM), shooterRollerDiameter);
    SmartDashboard.putNumber("AAAAAAAAARPMLV", shooterLinearVelocity.in(MetersPerSecond));

  }

  public double finalSetTargetRPM;

  private AngularAcceleration targetAcceleration = RotationsPerSecondPerSecond.of(500);

  private int tst = 0;

  public boolean endShooterCommand;

  public boolean hubIsActive;

  private boolean shootUsingDistance;

  public boolean bypassShootInterlocks = true;

  public boolean isShootOnTheMove;

  public boolean isShootUsingDistance() {
    return shootUsingDistance;
  }

  public void setShootUsingDistance(boolean autoShoot) {
    this.shootUsingDistance = autoShoot;
    finalSetTargetRPM = isShootUsingDistance() ? autoSetTargetRPM : manualSetTargetRPM;

  }

  public Command setShootUsingDistanceCommand(boolean on) {
    return Commands.sequence(
        Commands.runOnce(() -> shootUsingDistance = on),
        Commands.runOnce(() -> finalSetTargetRPM = isShootUsingDistance() ? autoSetTargetRPM : manualSetTargetRPM));

  }

  public TripleShooterSubsystem(boolean showData) {
    this.showData = showData;
    leftMotor = new TalonFX(CANIDConstants.leftShooterID, CanbusConstants.kCANivoreCANBus);
    middleMotor = new TalonFX(CANIDConstants.centerShooterID, CanbusConstants.kCANivoreCANBus);
    rightMotor = new TalonFX(CANIDConstants.rightShooterID, CanbusConstants.kCANivoreCANBus);

    Configs.Shooter.configureLeftMotor(leftMotor, InvertedValue.Clockwise_Positive);
    Configs.Shooter.configureMiddleMotor(middleMotor, InvertedValue.Clockwise_Positive);
    Configs.Shooter.configureRightMotor(rightMotor, InvertedValue.Clockwise_Positive);

    setShootUsingDistance(false);
    if (showData)
      SmartDashboard.putData(this);

    if (!statusL.isOK()) {
      DogLog.log("Left Shooter", "Could not apply configs, error code: " + statusL.toString());
    }
    if (!statusM.isOK()) {
      DogLog.log("Middle Shooter", "Could not apply configs, error code: " + statusL.toString());
    }
    if (!statusR.isOK()) {
      DogLog.log("Right Shooter", "Could not apply configs, error code: " + statusL.toString());
    }

  }

  public void runVelocityVoltage(TalonFX motor) {
    motor.setControl(
        velocityVoltage
            .withVelocity(RPM.of(finalSetTargetRPM))
            .withAcceleration(targetAcceleration)
            .withSlot(0)
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
            .withVelocity(manualSetTargetRPM / 60));
  }

  public void setPercentOutput(TalonFX motor, double percentOutput) {
    motor.setControl(
        voltageRequest
            .withOutput(Volts.of(percentOutput * 12.0)));
  }

  public void stopAllShooters() {
    setPercentOutput(leftMotor, 0.0);
    setPercentOutput(middleMotor, 0.0);
    setPercentOutput(rightMotor, 0.0);
    disableAllShooters();
    endShooterCommand = true;
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

  public void setManualTargetVelocity(double RPM) {
    manualSetTargetRPM = RPM;
  }

  public Command setManualTargetVelocityCommand(AngularVelocity vel) {
    return Commands.runOnce(() -> setManualTargetVelocity(vel.in(RPM)));
  }

  public Command changeFinalTargetVelocityCommand(double rpm) {
    return Commands.runOnce(() -> changeFinalTargetVelocity(rpm));
  }

  public void changeFinalTargetVelocity(double rpm) {
    finalSetTargetRPM += rpm;
    SmartDashboard.putNumber("TGTrpm", manualSetTargetRPM);
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
    return RobotBase.isSimulation() ||
        (isVelocityWithinTolerance(leftMotor) || !leftMotorActive)
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
    initSendable(builder, rightMotor, "Right");
    builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
        null);
    builder.addBooleanProperty("Use Distance For RPM", () -> isShootUsingDistance(), null);

    builder.addDoubleProperty("Voltage Target RPM", () -> velocityVoltage.getVelocityMeasure().in(RPM), null);
    builder.addDoubleProperty("Manual Target RPM", () -> manualSetTargetRPM, null);
    builder.addDoubleProperty("Auto Target RPM", () -> autoSetTargetRPM, null);
    builder.addDoubleProperty("Final Target RPM", () -> finalSetTargetRPM, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.log("Shooter/LeftRPM", leftMotor.getVelocity().getValue().in(RPM));
    Logger.log("Shooter/MiddleRPM", middleMotor.getVelocity().getValue().in(RPM));
    Logger.log("Shooter/RightRPM", rightMotor.getVelocity().getValue().in(RPM));
    Logger.log("Shooter/LeftMotorAtSpeed", isVelocityWithinTolerance(leftMotor));
    Logger.log("Shooter/MiddleMotorAtSpeed", isVelocityWithinTolerance(middleMotor));
    Logger.log("Shooter/RightMotorAtSpeed", isVelocityWithinTolerance(rightMotor));
    Logger.log("Shooter/AllMotorsAtSpeed", allVelocityInTolerance());

    Logger.log("Shooter/UseDistForRPM", isShootUsingDistance());
    Logger.log("Shooter/FinalTargetRPM", finalSetTargetRPM);
    Logger.log("Shooter/AutoTargetRPM", autoSetTargetRPM);
    Logger.log("Shooter/ManualTargetRPM", manualSetTargetRPM);
    Logger.log("Shooter/LeftAmps", leftMotor.getStatorCurrent().getValue().in(Amps));
    Logger.log("Shooter/MiddleAmps", middleMotor.getStatorCurrent().getValue().in(Amps));
    Logger.log("Shooter/RightAmps", leftMotor.getStatorCurrent().getValue().in(Amps));
    Logger.log("Shooter/RightVolts", leftMotor.getMotorVoltage().getValue().in(Volts));
    Logger.log("Shooter/MilddleVolts", middleMotor.getMotorVoltage().getValue().in(Volts));
    Logger.log("Shooter/RightVolts", rightMotor.getMotorVoltage().getValue().in(Volts));

  }

  public void setDistanceToHub(double distance) {
    distanceToHub = Meters.of(distance);
  }

  public Distance getDistanceToHub() {
    return distanceToHub;
  }

  public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
    return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters) * 0.54);
  }

  public LinearVelocity getLinearExitVelocity() {
    return angularToLinearVelocity(RadiansPerSecond.of(finalSetTargetRPM), shooterRollerDiameter);
  }

}
