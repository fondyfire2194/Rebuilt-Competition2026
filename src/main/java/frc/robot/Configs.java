// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.IntakeSlideArmSubsystem;

import com.revrobotics.spark.config.SparkMaxConfig;

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
          .smartCurrentLimit(40);
    }
  }

  public static final class IntakeArm {

    public static final SparkMaxConfig intakeSlideConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the intake motor
      intakeSlideConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(5)
          .smartCurrentLimit(40);
      intakeSlideConfig
          .inverted(false)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(60);

      intakeSlideConfig.encoder
          .positionConversionFactor(IntakeSlideArmSubsystem.positionConversionFactor)
          .velocityConversionFactor(IntakeSlideArmSubsystem.velocityConversionFactor);

      intakeSlideConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
      intakeSlideConfig.softLimit.forwardSoftLimit(IntakeSlideArmSubsystem.minDistance.in(Inches))
          .reverseSoftLimit(IntakeSlideArmSubsystem.minDistance.in(Inches))
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimitEnabled(true);

      intakeSlideConfig.signals.primaryEncoderPositionPeriodMs(10);

    }
  }

  public static final class Feeder {

    public static final SparkMaxConfig feederBeltConfig = new SparkMaxConfig();
    static { // Configure basic setting of the feeder belt motor
      feederBeltConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(1.0)
          .smartCurrentLimit(60);
    }

    public static final SparkMaxConfig feederRollerConfig = new SparkMaxConfig();
    static { // Configure basic setting of the feeder belt motor
      feederRollerConfig
          .inverted(true)
          .idleMode(IdleMode.kCoast)
          .openLoopRampRate(1.0)
          .smartCurrentLimit(60);
    }
  }
}
