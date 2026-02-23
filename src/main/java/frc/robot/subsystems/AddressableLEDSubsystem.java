// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AddressableLEDSubsystem extends SubsystemBase {
  /** Creates a new AddressableLEDSubsystem. */
  public boolean hubIsActive;
  public boolean fiveSecondWarningEndOfPickup;
  public boolean fiveSecondWarningEndOfShoot;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  // Create the buffer
  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(60);

  // Create the view for the section of the strip on the left side of the robot.
  // This section spans LEDs from index 0 through index 59, inclusive.
  AddressableLEDBufferView m_left = m_buffer.createView(0, 29);

  // The section of the strip on the right side of the robot.
  // This section spans LEDs from index 30 through index 59, inclusive.
  // This view is reversed to cancel out the serpentine arrangement of the
  // physical LED strip on the robot.
  AddressableLEDBufferView m_right = m_buffer.createView(30, 59).reversed();
  // Create an LED pattern that sets the entire strip to solid red
  LEDPattern red = LEDPattern.solid(Color.kRed);

  LEDPattern yellow = LEDPattern.solid(Color.kYellow);

  LEDPattern green = LEDPattern.solid(Color.kGreen);

  // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
  LEDPattern blinkGreen = green.blink(Seconds.of(.5));
  LEDPattern blinkRed = red.blink(Seconds.of(.5));

  public Trigger setGreen;
  public Trigger setGreenBlink;
  public Trigger setRed;
  public Trigger setRedBlink;

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
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));

    setGreen = new Trigger(
        () -> (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) && hubIsActive);
    setGreenBlink = new Trigger(() -> fiveSecondWarningEndOfShoot);
    setRed = new Trigger(() -> !hubIsActive);
    setRedBlink = new Trigger(() -> fiveSecondWarningEndOfPickup);

    setGreen.onTrue(runPattern(green));
    setGreenBlink.onTrue(runPattern(blinkGreen));

    setRed.onTrue(runPattern(red));
    setRedBlink.onTrue(runPattern(blinkRed));

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

    SmartDashboard.putBoolean("HUBACTIVE", hubIsActive);
    SmartDashboard.putBoolean("HUBWarningShoot", fiveSecondWarningEndOfShoot);
    SmartDashboard.putBoolean("HUBWarningPickup", fiveSecondWarningEndOfPickup);

    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }

}
