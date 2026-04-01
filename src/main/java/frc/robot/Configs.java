// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIDConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.Intake4BarArmSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Configs {

  private static final double nominalVoltage = 12.0;

  public static final class Intake {

    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the intake motor
      intakeConfig
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(0.5)
          .closedLoopRampRate(.25)
          .smartCurrentLimit(80);
      intakeConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for speed control. We don't need to pass a closed loop
          // slot, as it will default to slot 0.
          .p(0.00005)
          .i(0)
          .d(0)
          .outputRange(-1, 1).feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

    }
  }

  public static final class IntakeArm {

    public static final SparkMaxConfig armConfig1 = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig2 = new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor

      armConfig1
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60);

      armConfig1.encoder
          .positionConversionFactor(Intake4BarArmSubsystem.positionConversionFactor)
          .velocityConversionFactor(Intake4BarArmSubsystem.velocityConversionFactor);

      armConfig1.softLimit.forwardSoftLimit(Intake4BarArmSubsystem.maxAngle.in(Radians))
          .reverseSoftLimit(Intake4BarArmSubsystem.minAngle.in(Radians))
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);

      armConfig1.signals.primaryEncoderPositionPeriodMs(10);

    }

    static {
      armConfig2
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(60);
      armConfig2
          .follow(CANIDConstants.intakeArmID, true);

    }

  }

  public static final class Hood {

    public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the intake motor
      hoodConfig
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(5)
          .closedLoopRampRate(.25)
          .smartCurrentLimit(40);

      hoodConfig.encoder
          .positionConversionFactor(HoodSubsystem.degreesPerMotorRev)
          .velocityConversionFactor(HoodSubsystem.degreesPerMotorRev / 60);

      hoodConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed loop
          // slot, as it will default to slot 0.
          .p(0.2)
          .i(0)
          .d(0)
          .outputRange(-.75, .75);

      hoodConfig.softLimit.forwardSoftLimit(HoodSubsystem.kMaxPosition.in(Degrees))
          .reverseSoftLimit(HoodSubsystem.kMinPosition.in(Degrees))
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);

      hoodConfig.signals.primaryEncoderPositionPeriodMs(10);

    }

  }

  public static final class Feeder {

    public static final SparkMaxConfig feederBeltConfig = new SparkMaxConfig();
    static { // Configure basic setting of the feeder belt motor
      feederBeltConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(.25)
          .closedLoopRampRate(.05)
          .smartCurrentLimit(80);
      feederBeltConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for speed control. We don't need to pass a closed loop
          // slot, as it will default to slot 0.
          .p(0.0001)
          .i(0)
          .d(0)
          .outputRange(-1, 1);

      feederBeltConfig.closedLoop.feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

    }

    public static final SparkMaxConfig feederRollerConfig = new SparkMaxConfig();
    static { // Configure basic setting of the feeder belt motor
      feederRollerConfig
          .inverted(false)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(.1)
          .closedLoopRampRate(.75)
          .smartCurrentLimit(80);

      feederRollerConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for speed control. We don't need to pass a closed loop
          // slot, as it will default to slot 0.
          .p(0.0001)
          .i(0)
          .d(0)
          .outputRange(-1, 1);

      feederRollerConfig.closedLoop.feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);
    }
  }

  public static final class Shooter {

    public static void configureLeftMotor(TalonFX motor, InvertedValue invertDirection) {
      final TalonFXConfiguration leftConfigs = new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(invertDirection)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withStatorCurrentLimitEnable(true));

      /*
       * Voltage-based velocity requires a velocity feed forward to account for the
       * back-emf of the motor
       */
      leftConfigs.Slot0.kS = 0.05; // To account for friction, add 0.1 V of static feedforward
      leftConfigs.Slot0.kV = 0.115; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
      // volts / rotation per second
      leftConfigs.Slot0.kP = 0.2; // An error of 1 rotation per second results in 0.11 V output
      leftConfigs.Slot0.kI = 0; // No output for integrated error
      leftConfigs.Slot0.kD = 0; // No output for error derivative
      // Peak output of 10 volts
      leftConfigs.Voltage.withPeakForwardVoltage(Volts.of(8))
          .withPeakReverseVoltage(Volts.of(-8));

      /*
       * Torque-based velocity does not require a velocity feed forward, as torque
       * will accelerate the rotor up to the desired velocity by itself
       */

      leftConfigs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
      leftConfigs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
      leftConfigs.Slot1.kI = 0; // No output for integrated error
      leftConfigs.Slot1.kD = 0; // No output for error derivative
      // Peak output of 40 A
      leftConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
          .withPeakReverseTorqueCurrent(Amps.of(-40));
      /* Retry config apply up to 5 times, report if failure */

      for (int i = 0; i < 5; ++i) {
        TripleShooterSubsystem.statusL = motor.getConfigurator().apply(leftConfigs);
        if (TripleShooterSubsystem.statusL.isOK())
          break;
      }

      // TunableTalonFXPid.create("TuneLeftShooter", motor, leftConfigs);
    }

    public static void configureMiddleMotor(TalonFX motor, InvertedValue invertDirection) {
      final TalonFXConfiguration middleConfigs = new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(invertDirection)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withStatorCurrentLimitEnable(true));

      /*
       * Voltage-based velocity requires a velocity feed forward to account for the
       * back-emf of the motor
       */
      middleConfigs.Slot0.kS = 0.05; // To account for friction, add 0.1 V of static feedforward
      middleConfigs.Slot0.kV = 0.115; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
      // volts / rotation per second
      middleConfigs.Slot0.kP = 0.2; // An error of 1 rotation per second results in 0.11 V output
      middleConfigs.Slot0.kI = 0; // No output for integrated error
      middleConfigs.Slot0.kD = 0; // No output for error derivative
      // Peak output of 10 volts
      middleConfigs.Voltage.withPeakForwardVoltage(Volts.of(8))
          .withPeakReverseVoltage(Volts.of(-8));

      /*
       * Torque-based velocity does not require a velocity feed forward, as torque
       * will accelerate the rotor up to the desired velocity by itself
       */

      middleConfigs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
      middleConfigs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
      middleConfigs.Slot1.kI = 0; // No output for integrated error
      middleConfigs.Slot1.kD = 0; // No output for error derivative
      // Peak output of 40 A
      middleConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
          .withPeakReverseTorqueCurrent(Amps.of(-40));
      for (int i = 0; i < 5; ++i) {
        TripleShooterSubsystem.statusM = motor.getConfigurator().apply(middleConfigs);
        if (TripleShooterSubsystem.statusM.isOK())
          break;
      }

      // TunableTalonFXPid.create("TuneMiddleShooter", motor, middleConfigs);
    }

    public static void configureRightMotor(TalonFX motor, InvertedValue invertDirection) {
      final TalonFXConfiguration rightConfigs = new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(invertDirection)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(80))
                  .withStatorCurrentLimitEnable(true));

      /*
       * Voltage-based velocity requires a velocity feed forward to account for the
       * back-emf of the motor
       */
      rightConfigs.Slot0.kS = 0.05; // To account for friction, add 0.1 V of static feedforward
      rightConfigs.Slot0.kV = 0.115; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
      // volts / rotation per second
      rightConfigs.Slot0.kP = 0.2; // An error of 1 rotation per second results in 0.11 V output
      rightConfigs.Slot0.kI = 0; // No output for integrated error
      rightConfigs.Slot0.kD = 0; // No output for error derivative
      // Peak output of 10 volts
      rightConfigs.Voltage.withPeakForwardVoltage(Volts.of(8))
          .withPeakReverseVoltage(Volts.of(-8));

      /*
       * Torque-based velocity does not require a velocity feed forward, as torque
       * will accelerate the rotor up to the desired velocity by itself
       */

      rightConfigs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
      rightConfigs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
      rightConfigs.Slot1.kI = 0; // No output for integrated error
      rightConfigs.Slot1.kD = 0; // No output for error derivative
      // Peak output of 40 A
      rightConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
          .withPeakReverseTorqueCurrent(Amps.of(-40));
      for (int i = 0; i < 5; ++i) {
        TripleShooterSubsystem.statusR = motor.getConfigurator().apply(rightConfigs);
        if (TripleShooterSubsystem.statusR.isOK())
          break;
      }

      // TunableTalonFXPid.create("TuneRightShooter", motor, rightConfigs);
    }

  }
}