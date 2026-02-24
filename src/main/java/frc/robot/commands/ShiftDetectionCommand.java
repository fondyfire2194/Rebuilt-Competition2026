// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShiftDetectionCommand extends Command {
  /** Creates a new ShiftDetectionCommand. */
  private final TripleShooterSubsystem m_shooter;
  private final AddressableLEDSubsystem m_leds;
  private final double fuelToHubTime = 2.25;

  private final double firstShiftStartTime = 130;
  private final double secondShiftStartTime = 105;
  private final double thirdShiftStartTime = 80;
  private final double fourthShiftStartTime = 55;
  private final double endGameStartTime = 30;

  private final double driverWarningTime = 5;
  private int shiftNumber = 0;
  

  double matchTime;

  private boolean hubIsActive;
  private boolean blueActiveFirst;

  // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  public ShiftDetectionCommand(TripleShooterSubsystem shooter, AddressableLEDSubsystem leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_leds = leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    matchTime = DriverStation.getMatchTime();
    m_leds.gameData = DriverStation.getGameSpecificMessage();// letter is high auto score alliance

    hubIsActive = isHubActive();
    m_shooter.hubIsActive = hubIsActive;
    m_leds.hubIsActive = hubIsActive;

    m_leds.fiveSecondWarningEndOfShoot = hubIsActive && getFiveSecondWarning();
    m_leds.fiveSecondWarningEndOfPickup = !hubIsActive && getFiveSecondWarning();
    m_leds.endGameWarning = getEndGameWarning();
    m_leds.inEndGame = getInEndGame();
    m_leds.endOfMatch = endOfMatch();

    SmartDashboard.putNumber("MatchTime", matchTime);
    SmartDashboard.putNumber("ShiftNum", shiftNumber);
    SmartDashboard.putBoolean("BLUEFIRST", blueActiveFirst);
    SmartDashboard.putNumber("ShiftTimeLeft", getTimeLeftInShift());

    SmartDashboard.putBoolean("EndOfMatch", endOfMatch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.

    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (m_leds.gameData.isEmpty()) {
      return RobotBase.isReal();
    }
    blueActiveFirst = false;
    // game data gives auto high score alliance - they shoot shifts 2 and 4
    switch (m_leds.gameData.charAt(0)) {
      case 'R' -> blueActiveFirst = true;
      case 'B' -> blueActiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    m_leds.currentAllianceShootActive = switch (alliance.get()) {
      case Red -> !blueActiveFirst;
      case Blue -> blueActiveFirst;
    };

    if (m_leds.forceFirstAlliance) {
     m_leds. currentAllianceShootActive = m_leds.blueActiveFirst;
    }

    if (matchTime >= firstShiftStartTime) {
      // transition shift
      return true;

    } else if (matchTime < firstShiftStartTime && matchTime > secondShiftStartTime) {

      // Shift 1

      shiftNumber = 1;

      return m_leds.currentAllianceShootActive;

    } else if (matchTime < secondShiftStartTime && matchTime > thirdShiftStartTime) {
      // Shift 2

      shiftNumber = 2;

      return !m_leds.currentAllianceShootActive;

    } else if (matchTime < thirdShiftStartTime && matchTime > fourthShiftStartTime) {
      // Shift 3
      {
        shiftNumber = 3;
      }
      return m_leds.currentAllianceShootActive;

    } else if (matchTime < fourthShiftStartTime && matchTime > endGameStartTime) {
      // Shift 3
      {
        shiftNumber = 4;
      }
      return !m_leds.currentAllianceShootActive;

    } else {
      shiftNumber = 5;
      // End game, hub always active.
      return true;
    }
  }

  public boolean getFiveSecondWarning() {
    return shiftNumber == 1 && matchTime < secondShiftStartTime + driverWarningTime
        || shiftNumber == 2 && matchTime < thirdShiftStartTime + driverWarningTime
        || shiftNumber == 3 && matchTime < fourthShiftStartTime + driverWarningTime;
  }

  public boolean getEndGameWarning() {
    return shiftNumber == 4 && matchTime < endGameStartTime + driverWarningTime;
  }

  public boolean getInEndGame() {
    return shiftNumber == 5;
  }

  public boolean endOfMatch() {
    return shiftNumber == 5 && matchTime < 2;
  }

  public double getTimeLeftInShift() {
    if (DriverStation.isTeleopEnabled()) {
      switch (shiftNumber) {
        case 0:
          return matchTime - firstShiftStartTime;
        case 1:
          return matchTime - secondShiftStartTime;
        case 2:
          return matchTime - thirdShiftStartTime;
        case 3:
          return matchTime - fourthShiftStartTime;
        case 4:
          return matchTime - endGameStartTime;
        case 5:
          return matchTime;
        default:
          return 0;
      }
    } else
      return 0;
  }

}
