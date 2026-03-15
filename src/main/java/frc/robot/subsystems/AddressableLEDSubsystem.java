// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AddressableLEDSubsystem extends SubsystemBase {
  /** Creates a new AddressableLEDSubsystem. */
  public boolean hubIsActive;
  public boolean fiveSecondWarningEndOfPickup;
  public boolean fiveSecondWarningEndOfShoot;
  public boolean endGameWarning;
  public boolean inEndGame;
  public boolean endOfMatch;
  public String gameData = "placeholder";
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private static int LED_STRIP_LENGTH = 60;
  // Create the buffer
  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(LED_STRIP_LENGTH);

  // Create the view for the section of the strip on the left side of the robot.
  // This section spans LEDs from index 0 through index 59, inclusive.
  AddressableLEDBufferView m_left = m_buffer.createView(0, 29);

  // The section of the strip on the right side of the robot.
  // This section spans LEDs from index 30 through index 59, inclusive.
  // This view is reversed to cancel out the serpentine arrangement of the
  // physical LED strip on the robot.
  AddressableLEDBufferView m_right = m_buffer.createView(30, 59).reversed();
  // Create an LED pattern that sets the entire strip to solid red

  /** A pattern that turns off all LEDs. */
  LEDPattern kOff = LEDPattern.solid(Color.kBlack);
  LEDPattern red = LEDPattern.solid(Color.kRed);

  LEDPattern yellow = LEDPattern.solid(Color.kYellow);

  LEDPattern green = LEDPattern.solid(Color.kGreen);

  private LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1. / LED_STRIP_LENGTH);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip,
  // moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
  LEDPattern blinkGreen = green.blink(Seconds.of(.5));
  LEDPattern blinkRed = red.blink(Seconds.of(.5));
  LEDPattern blinkYellow = yellow.blink(Seconds.of(.5));

  public Trigger setGreen;
  public Trigger setGreenBlink;
  public Trigger setRed;
  public Trigger setRedBlink;

  public Trigger setYellow;
  public Trigger setYellowBlink;
  public Trigger endOfMatchTrigger;

  private Timer timer = new Timer();
  private Trigger timerTrigger;
  public boolean forceFirstAlliance;
  public boolean blueActiveFirst;
  public boolean currentAllianceShootActive;

  public AddressableLEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!

    setDefaultCommand(
        runPattern(kOff).ignoringDisable(true));

    CommandScheduler.getInstance().schedule(
        Commands.sequence(
            runPattern(m_scrollingRainbow),
            Commands.waitSeconds(10),
            runPattern(kOff))
            .ignoringDisable(true));

    setGreen = new Trigger(
        () -> (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) && hubIsActive);

    setGreenBlink = new Trigger(() -> fiveSecondWarningEndOfShoot);

    setYellow = new Trigger(() -> inEndGame);
    setYellowBlink = new Trigger(() -> endGameWarning);

    setRed = new Trigger(() -> !hubIsActive);
    setRedBlink = new Trigger(() -> fiveSecondWarningEndOfPickup);

    setGreen.onTrue(runPattern(green));
    setGreenBlink.onTrue(runPattern(blinkGreen));
    setGreenBlink.onFalse(runPattern(yellow));

    setYellow.onTrue(runPattern(yellow));
    setYellowBlink.onTrue(runPattern(blinkYellow));
    setYellowBlink.onFalse(runPattern(green));

    setRed.onTrue(runPattern(red));
    setRedBlink.onTrue(runPattern(blinkRed));

    timerTrigger = new Trigger(() -> timer.get() > 10.);
    timerTrigger.onTrue(runPattern(kOff).ignoringDisable(true));

    endOfMatchTrigger = new Trigger(() -> endOfMatch = true);

    endOfMatchTrigger.onTrue(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    timer.reset();
    timer.start();

  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to
    /**
     * if hub is active and not in 5 second warning show solid green
     * if hub is active and in 5 second warning show flashing green
     * 
     * if hub is not active and not in 5 second warning show blue
     * if hub is not active and in 5 second warning show flashing blue
     * 
     */

    // Logger.log("LEDS/WarningShoot", fiveSecondWarningEndOfShoot);
    // Logger.log("LEDS/WarningPickup", fiveSecondWarningEndOfPickup);
    // Logger.log("LEDS/WarningEndGame", endGameWarning);
    // Logger.log("LEDS/AllianceShootActive", currentAllianceShootActive);
    // Logger.log("LEDS/HiAuto-GameData", gameData);
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return Commands.run(() -> pattern.applyTo(m_buffer), this);
  }

  /**
   * Gets the length of the LED strips.
   * 
   * @return length of the strips
   */
  public int getStripLength() {
    return LED_STRIP_LENGTH;
  }

  public Command toggleGameData() {
    return Commands.sequence(
        Commands.runOnce(() -> forceFirstAlliance = true),
        new ConditionalCommand(
            Commands.runOnce(() -> blueActiveFirst = true),
            Commands.runOnce(() -> blueActiveFirst = false),
            () -> blueActiveFirst == false));
  }

}
