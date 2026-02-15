// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TripleShooterSubsystem;

/** Add your docs here. */

public class LoopEvents {
    CommandSwerveDrivetrain m_drivetrain;
    TripleShooterSubsystem m_shooter;
    EventLoop m_loop;

    public LoopEvents(CommandSwerveDrivetrain drivetrain, TripleShooterSubsystem shooter, EventLoop loop) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_loop = loop;
    }

    public void init() {
        BooleanEvent hitBump = new BooleanEvent(
                m_loop, () -> Math.abs(m_drivetrain.getPigeon2().getPitch().getValueAsDouble()) > 10)
                .debounce(.2);


    }

}
