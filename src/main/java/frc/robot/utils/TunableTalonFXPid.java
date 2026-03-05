// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class TunableTalonFXPid {

  public static void create(String key, TalonFX motor, TalonFXConfiguration defaultConfig) {
    DogLog.tunable(key + "/kP0", defaultConfig.Slot0.kP,
        newP -> motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP)));
    DogLog.tunable(key + "/kI0", defaultConfig.Slot0.kI,
        newI -> motor.getConfigurator().apply(defaultConfig.Slot0.withKI(newI)));
    DogLog.tunable(key + "/kD0", defaultConfig.Slot0.kD,
        newD -> motor.getConfigurator().apply(defaultConfig.Slot0.withKD(newD)));
    DogLog.tunable(key + "/kS0", defaultConfig.Slot0.kS,
        newS -> motor.getConfigurator().apply(defaultConfig.Slot0.withKS(newS)));
    // DogLog.tunable(key + "/kV0", defaultConfig.Slot0.kV,
    //      newV -> motor.getConfigurator().apply(defaultConfig.Slot0.withKV(newV)));
    DogLog.tunable(key + "/kA0", defaultConfig.Slot0.kA,
        newA -> motor.getConfigurator().apply(defaultConfig.Slot0.withKA(newA)));
    DogLog.tunable(key + "/kG0", defaultConfig.Slot0.kG,
        newG -> motor.getConfigurator().apply(defaultConfig.Slot0.withKG(newG)));
    // slot 1
    DogLog.tunable(key + "/kP1", defaultConfig.Slot1.kP,
        newP -> motor.getConfigurator().apply(defaultConfig.Slot1.withKP(newP)));
    DogLog.tunable(key + "/kI1", defaultConfig.Slot1.kI,
        newI -> motor.getConfigurator().apply(defaultConfig.Slot1.withKI(newI)));
    DogLog.tunable(key + "/kD1", defaultConfig.Slot1.kD,
        newD -> motor.getConfigurator().apply(defaultConfig.Slot1.withKD(newD)));
   // DogLog.tunable(key + "/kS1", defaultConfig.Slot1.kS,
    //     newS -> motor.getConfigurator().apply(defaultConfig.Slot1.withKS(newS)));
    // DogLog.tunable(key + "/kV1", defaultConfig.Slot1.kV,
    //     newV -> motor.getConfigurator().apply(defaultConfig.Slot1.withKV(newV)));
    DogLog.tunable(key + "/kA1", defaultConfig.Slot1.kA,
        newA -> motor.getConfigurator().apply(defaultConfig.Slot1.withKA(newA)));
    // DogLog.tunable(key + "/kG1", defaultConfig.Slot1.kG,
    //     newG -> motor.getConfigurator().apply(defaultConfig.Slot0.withKG(newG)));
  }

    private TunableTalonFXPid() {
    }
}