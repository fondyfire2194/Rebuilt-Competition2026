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
  private boolean currentAllianceShootActive;

  double matchTime;
  String gameData;
  private boolean hubIsActive;
  private boolean blueActiveFirst;

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
    gameData = DriverStation.getGameSpecificMessage();// letter is high auto score alliance

    if (RobotBase.isSimulation())
      gameData = "R";
    hubIsActive = isHubActive();
    m_shooter.hubIsActive = hubIsActive;
    m_leds.hubIsActive = hubIsActive;

    m_leds.fiveSecondWarningEndOfShoot = hubIsActive && getFiveSecondWarningEnd();
    m_leds.fiveSecondWarningEndOfPickup = !hubIsActive && getFiveSecondWarningEnd();

    SmartDashboard.putNumber("HubMatchtime", matchTime);
    SmartDashboard.putString("HubGamedata", gameData);
    SmartDashboard.putNumber("HubShiftNum", shiftNumber);
    SmartDashboard.putBoolean("HubBLUEFIRST", blueActiveFirst);

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
    if (gameData.isEmpty()) {
      return RobotBase.isReal();
    }
    blueActiveFirst = false;
    // game data gives auto high score alliance - they shoot shifts 2 and 4
    switch (gameData.charAt(0)) {
      case 'R' -> blueActiveFirst = true;
      case 'B' -> blueActiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    currentAllianceShootActive = switch (alliance.get()) {
      case Red -> !blueActiveFirst;
      case Blue -> blueActiveFirst;
    };

    if (matchTime >= firstShiftStartTime) {
      // transition shift
      return true;

    } else if (matchTime < firstShiftStartTime && matchTime > secondShiftStartTime) {

      // Shift 1

      shiftNumber = 1;

      return currentAllianceShootActive;

    } else if (matchTime < secondShiftStartTime && matchTime > thirdShiftStartTime) {
      // Shift 2

      shiftNumber = 2;

      return !currentAllianceShootActive;

    } else if (matchTime < thirdShiftStartTime && matchTime > fourthShiftStartTime) {
      // Shift 3
      {
        shiftNumber = 3;
      }
      return currentAllianceShootActive;

    } else if (matchTime < fourthShiftStartTime && matchTime > endGameStartTime) {
      // Shift 3
      {
        shiftNumber = 4;
      }
      return !currentAllianceShootActive;

    } else {
      // End game, hub always active.
      return true;
    }
  }

  public boolean getFiveSecondWarningEnd() {
    return shiftNumber == 1 && matchTime < secondShiftStartTime + driverWarningTime
        || shiftNumber == 2 && matchTime < thirdShiftStartTime + driverWarningTime
        || shiftNumber == 3 && matchTime < fourthShiftStartTime + driverWarningTime;
       // || shiftNumber == 4 && matchTime < endGameStartTime + driverWarningTime;
  }

}
